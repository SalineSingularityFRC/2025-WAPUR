package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex motor1;
    private SparkFlex motor2;
    private SparkFlexConfig motor1Config;
    private SparkFlexConfig motor2Config;

    public IntakeSubsystem() {
        motor1 = new SparkFlex(50, MotorType.kBrushless);
        motor2 = new SparkFlex(61, MotorType.kBrushless);
        motor1Config = new SparkFlexConfig();
        motor2Config = new SparkFlexConfig();

        motor1Config.idleMode(IdleMode.kBrake);
        motor2Config.idleMode(IdleMode.kBrake);

        motor1Config.closedLoop.minOutput(-1).maxOutput(1).pid(1,0,0);
        motor2Config.closedLoop.minOutput(-1).maxOutput(1).pid(1,0,0);

        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runMotors(double speed) {
        return runEnd(() -> {
            motor1.set(speed);
            motor2.set(-speed);
        }, () -> {
            motor1.set(0);
            motor2.set(0);
        });
    }
}
