package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    public TalonFX motor1 = new TalonFX(Constants.CANID.IntakeSubsystem.LINTAKEMOTOR);
    public TalonFX motor2 = new TalonFX(Constants.CANID.IntakeSubsystem.RINTAKEMOTOR);
    public TalonFXConfiguration motor1Config;
    public TalonFXConfiguration motor2Config;

    public IntakeSubsystem() {
        motor1Config = new TalonFXConfiguration();
        motor2Config = new TalonFXConfiguration();

        motor1Config.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(4).withStatorCurrentLimitEnable(false);
        motor1Config.Slot0.kP = Constants.PID.Intake.kP.getValue();
        motor1Config.Slot0.kI = Constants.PID.Intake.kI.getValue();
        motor1Config.Slot0.kD = Constants.PID.Intake.kD.getValue();

        motor2Config.CurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(4).withStatorCurrentLimitEnable(false);
        motor2Config.Slot0.kP = Constants.PID.Intake.kP.getValue();
        motor2Config.Slot0.kI = Constants.PID.Intake.kI.getValue();
        motor2Config.Slot0.kD = Constants.PID.Intake.kD.getValue();

        motor1.getConfigurator().apply(motor1Config);
        motor2.getConfigurator().apply(motor2Config);
    }

    public Command RunMotors(double speed){
        return runEnd(() -> {
            motor1.set(speed);
            motor2.set(-speed);
        },
        () -> {
            motor1.set(0);
            motor2.set(0);
        });
    }
}
