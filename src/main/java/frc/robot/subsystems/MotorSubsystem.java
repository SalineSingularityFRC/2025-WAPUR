package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
    private SparkFlex motor1;
    private SparkFlex motor2;
    private SparkFlexConfig motor1Config;
    public MotorSubsystem() {
        motor1 = new SparkFlex(61, SparkFlex.MotorType.kBrushless);
        motor2 = new SparkFlex(50, SparkFlex.MotorType.kBrushless);

        motor1Config = new SparkFlexConfig();
        motor1Config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).minOutput(-1).maxOutput(1).pid(1,0,0);
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runMotors(double speed) {
        return runEnd(() -> {
            motor1.set(speed);
            motor2.set(speed);
        }, () -> {
            motor1.set(0);
            motor2.set(0);
        });
    }
}
