// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TestShooterSubsytem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private TestShooterSubsytem shooterSubsystem = new TestShooterSubsytem();
    private Pigeon2 gyro = new Pigeon2(20, "drivetrain");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double rotationRate = -joystick.getRawAxis(1) * MaxAngularRate;
    private PIDController rotationController = new PIDController(Constants.PID.Drive.HEADING_CORRECTION_KP,
            Constants.PID.Drive.HEADING_CORRECTION_KI, Constants.PID.Drive.HEADING_CORRECTION_KD);
    private SimpleMotorFeedforward rotationFeedforward = new SimpleMotorFeedforward(
            Constants.PID.Drive.HEADING_CORRECTION_KS, Constants.PID.Drive.HEADING_CORRECTION_KV);
    private boolean isRotating = false;
    private double pastRobotAngle = 0;
    private double pastRobotAngleDerivative = 0;
    private double currentRobotAngleDerivative = 0;

    public RobotContainer() {
        rotationController.setSetpoint(gyro.getYaw().getValueAsDouble());
        rotationController.setTolerance(Constants.PID.Drive.HEADING_CORRECTION_TOLERANCE);
        SmartDashboard.putNumber("MaxAngularRate", MaxAngularRate);
        configureBindings();
    }

    public void updateRotation() {
        double gyroYaw = gyro.getYaw().getValueAsDouble();
        currentRobotAngleDerivative = gyroYaw - pastRobotAngle;
        if (currentRobotAngleDerivative == 0) {
            currentRobotAngleDerivative = pastRobotAngleDerivative;
        }

        if ((pastRobotAngleDerivative > 0 && currentRobotAngleDerivative < 0) ||
                (pastRobotAngleDerivative < 0 && currentRobotAngleDerivative > 0)) {
            isRotating = false;
        }

        boolean outDeadband = Math.abs(joystick.getRightX()) >= 0.015 * MaxAngularRate; // Reference:
                                                                                          // -joystick.getRightX() < 0.1
                                                                                          // * MaxAngularRate
        SmartDashboard.putBoolean("WithinDeadband", outDeadband);
        SmartDashboard.putNumber("RightX", joystick.getRightX());
        SmartDashboard.putNumber("GyroYaw", gyro.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("RotationRate", rotationRate);
        SmartDashboard.putNumber("RotationControllerSetpoint", rotationController.getSetpoint());
        if (outDeadband || isRotating) {
            isRotating = true;
            rotationRate = -joystick.getRightX() * 2;
            rotationController.setSetpoint(gyroYaw);
            rotationController.reset();
        } else {
            double controllerCalculation = rotationController.calculate(gyroYaw);
            double feedforwardCalculation = rotationFeedforward.calculate(controllerCalculation);
            rotationRate = MathUtil.clamp(controllerCalculation + feedforwardCalculation, -0.5, 0.5);
            SmartDashboard.putNumber("controllerCalculation", controllerCalculation);
        }

        pastRobotAngle = gyroYaw;
        pastRobotAngleDerivative = currentRobotAngleDerivative;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(rotationRate) // Drive counterclockwise with negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));
        joystick.a().whileTrue(shooterSubsystem.runMotors(100));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on right bumper press
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
