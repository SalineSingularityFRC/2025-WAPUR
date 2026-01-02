// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = 1.52; // TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                    // desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.3).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private Pigeon2 gyro = new Pigeon2(20, "drivetrain");

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private Supplier<Pose2d> poseSupplier = () -> (drivetrain.getState().Pose);
    private Consumer<Pose2d> poseConsumer = pose -> {
        drivetrain.resetPose(pose);
    };
    private Supplier<ChassisSpeeds> chassisSupplier = () -> (drivetrain.getState().Speeds);
    private Consumer<ChassisSpeeds> chassisConsumer = speed -> {
        drivetrain.setControl(
                new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speed)
        );
        SmartDashboard.putNumber("TestChassisSpeed/vx", speed.vxMetersPerSecond);
        SmartDashboard.putNumber("TestChassisSpeed/vy", speed.vyMetersPerSecond);
        SmartDashboard.putNumber("TestChassisSpeed/omega", speed.omegaRadiansPerSecond);
    };
    private double rotationRate = -joystick.getRawAxis(1) * MaxAngularRate;
    private PIDController rotationController = new PIDController(Constants.PIDs.Drive.HEADING_CORRECTION_KP,
            Constants.PIDs.Drive.HEADING_CORRECTION_KI, Constants.PIDs.Drive.HEADING_CORRECTION_KD);
    private SimpleMotorFeedforward rotationFeedforward = new SimpleMotorFeedforward(
            Constants.PIDs.Drive.HEADING_CORRECTION_KS, Constants.PIDs.Drive.HEADING_CORRECTION_KV);
    private boolean isRotating = false;
    private double pastRobotAngle = 0;
    private double pastRobotAngleDerivative = 0;
    private double currentRobotAngleDerivative = 0;

    public RobotContainer() {
        rotationController.setSetpoint(gyro.getYaw().getValueAsDouble());
        rotationController.setTolerance(Constants.PIDs.Drive.HEADING_CORRECTION_TOLERANCE);
        SmartDashboard.putNumber("MaxAngularRate", MaxAngularRate);
        configureBindings();

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(poseSupplier, poseConsumer, chassisSupplier, chassisConsumer,
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                // Constants class
                        new PIDConstants(Constants.PIDGains.PathPlanner.translation.P,
                                Constants.PIDGains.PathPlanner.translation.I,
                                Constants.PIDGains.PathPlanner.translation.D), // Translation PID constants
                        new PIDConstants(Constants.PIDGains.PathPlanner.rotation.P,
                                Constants.PIDGains.PathPlanner.rotation.I,
                                Constants.PIDGains.PathPlanner.rotation.D) // Rotation PID constants
                ), config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, drivetrain);
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Forward");
    }
}
