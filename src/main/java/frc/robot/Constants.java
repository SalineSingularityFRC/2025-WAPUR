package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final class Modes {
    /*
     * Mode.REAL - if on a real robot
     * Mode.SIM - if on "Simulate Robot Code"
     * Mode.REPLAY - finds path to log file and puts in AdvantageScope (if open)
     */
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM; // Mode.REAL : Mode.REPLAY;
  }

  public static enum Mode {
    REAL, SIM, REPLAY
  }

  public static class CANID {
    public static class IntakeSubsystem{
      public static final int LINTAKEMOTOR = 5;
      public static final int RINTAKEMOTOR = 6;
    }
  }

  public static class PID{
    public static class Intake{
      public static final ConfigDouble kP = new ConfigDouble("Intake/kP", 1);
      public static final ConfigDouble kI = new ConfigDouble("Intake/kI", 0);
      public static final ConfigDouble kD = new ConfigDouble("Intake/kD", 0);
    }

    public static class Drive {
      public static ConfigDouble HEADING_CORRECTION_KP = new ConfigDouble("Drive/PID/Heading Correction kP", 1);
      public static ConfigDouble HEADING_CORRECTION_KI = new ConfigDouble("Drive/PID/Heading Correction kI", 0);
      public static ConfigDouble HEADING_CORRECTION_KD = new ConfigDouble("Drive/PID/Heading Correction kD", 0);
    }
  }

  public static class Elevator {
    public static ConfigInt FOLLOW_DUALENABLE = new ConfigInt("Elevator/DUALENABLE", 0);

    public static class MotorStuff {
      public static ConfigDouble kP = new ConfigDouble("Elevator/Primary Motor/kP", 1); // Due to kP changing name from "Processor/kP" to "Elevator/Primary Motor/kP", not sure if set value was different from default value
      public static ConfigDouble MIN_POWER = new ConfigDouble("Elevator/Primary Motor/Min Power", -1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Elevator/Primary Motor/Max Power", 1);

      public static ConfigDouble MAX_VELOCITY_RPM = new ConfigDouble("Elevator/Primary Motor/Max Velocity in rpm",
          2000);
      public static ConfigDouble MAX_ACCEL_RPM_PER_S = new ConfigDouble("Elevator/Primary Motor/Max Accel in rpm per s",
          200);
      public static ConfigInt MAX_CURRENT_IN_A = new ConfigInt("Elevator/Primary Motor/Max Current in A", 40);
      public static ConfigDouble VOLTAGE_COMPENSATION_IN_V = new ConfigDouble(
          "Elevator/Primary Motor/Voltage Compensation in V", 12);
      public static ConfigDouble MAX_CONTROL_ERROR_IN_COUNTS = new ConfigDouble(
          "Elevator/Primary Motor/Control Error Tolerance", 0.25);
    }

    public static class Heights {

      // From the ground (inches)
      public static ConfigDouble LOWEST_HEIGHT = new ConfigDouble("Elevator/Heights/Lowest Height", 12);
      public static ConfigDouble HIGHEST_HEIGHT = new ConfigDouble("Elevator/Positions/Highest Height", 72);
      public static ConfigDouble DEADZONE = new ConfigDouble("Elevator/Positions/Deadzone Intake See Elevator", 7.0);
    }

    public static class Positions {
      public static ConfigDouble FEED_STATION_COUNTS = new ConfigDouble("Elevator/Positions/Feed Station in counts",
          -2);
      public static ConfigDouble L1_COUNTS = new ConfigDouble("Elevator/Positions/L1 in counts", -2);
      public static ConfigDouble L2_COUNTS = new ConfigDouble("Elevator/Positions/L2 in counts", 23);
      public static ConfigDouble L3_COUNTS = new ConfigDouble("Elevator/Positions/L3 in counts", 53);
      public static ConfigDouble L4_COUNTS = new ConfigDouble("Elevator/Positions/L4 in counts", 97.5);
      public static ConfigDouble L4_COUNTS_ADDITIONAL_RAISE = new ConfigDouble(
          "Elevator/Positions/L4 additional raise in counts", 1);
    }

    public static class PrimaryMotor {
      public static ConfigDouble RAISE_SPEED = new ConfigDouble("Elevator/Primary Motor/RAISE_SPEED", 0.3);
      public static ConfigDouble L4_RAISE_SPEED = new ConfigDouble("Elevator/Primary Motor/L4_RAISE_SPEED", 0.1);
      public static ConfigDouble LOWER_SPEED = new ConfigDouble("Elevator/Primary Motor/LOWER_SPEED", 0.2);
      public static ConfigInt INVERTED = new ConfigInt("Elevator/Primary Motor/ INVERTED MOTOR", 1);
      public static ConfigInt CAN_ID = new ConfigInt("Elevator/Primary Motor/CAN ID", 40);
      public static ConfigDouble KPUP = new ConfigDouble("Elevator/Primary Motor/kPUP", 0.082);
      public static ConfigDouble KIUP = new ConfigDouble("Elevator/Primary Motor/kIUP", 0);
      public static ConfigDouble KDUP = new ConfigDouble("Elevator/Primary Motor/kDUP", 0);
      public static ConfigDouble KFUP = new ConfigDouble("Elevator/Primary Motor/kFUP", 0);
      public static ConfigDouble KPDOWN = new ConfigDouble("Elevator/Primary Motor/kPDOWN", 0.073);
      public static ConfigDouble KIDOWN = new ConfigDouble("Elevator/Primary Motor/kIDOWN", 0);
      public static ConfigDouble KDDOWN = new ConfigDouble("Elevator/Primary Motor/kDDOWN", 0);
      public static ConfigDouble KFDOWN = new ConfigDouble("Elevator/Primary Motor/kFDOWN", 0);
      public static ConfigDouble arbFF = new ConfigDouble("Elevator/Primary Motor/arbFF", 0);

      public static ConfigDouble MIN_POWER = new ConfigDouble("Elevator/Primary Motor/Min Power", -1);
      public static ConfigDouble MAX_POWER = new ConfigDouble("Elevator/Primary Motor/Max Power", 1);

      public static ConfigDouble MAX_VELOCITY_RPM = new ConfigDouble("Elevator/Primary Motor/Max Velocity in rpm",
          6784);
      public static ConfigDouble MAX_ACCEL_RPM_PER_S = new ConfigDouble("Elevator/Primary Motor/Max Accel in rpm per s",
          20000);
      public static ConfigInt MAX_CURRENT_IN_A = new ConfigInt("Elevator/Primary Motor/Max Current in A", 60);
      public static ConfigDouble VOLTAGE_COMPENSATION_IN_V = new ConfigDouble(
          "Elevator/Primary Motor/Voltage Compensation in V", 12);
      public static ConfigDouble MAX_CONTROL_ERROR_IN_COUNTS = new ConfigDouble(
          "Elevator/Primary Motor/Control Error Tolerance", 0.25);
    }

    public static class SecondaryMotor {
      public static ConfigInt CAN_ID = new ConfigInt("Elevator/Secondary Motor/CAN ID", 41);
    }
  }

  public abstract static class Config<T> {
    public final String name;
    public final T defaultValue;

    Config(String name, T defaultValue) {
      this.name = name;
      this.defaultValue = defaultValue;
    }

    abstract public T getValue();
  }

  public static class ConfigDouble extends Config<Double> {
    public ConfigDouble(String name, double defaultValue) {
      super(name, defaultValue);

      // Make sure that it shows up in the Preferences
      Preferences.initDouble(name, defaultValue);
    }

    @Override
    public Double getValue() {
      return Preferences.getDouble(name, defaultValue);
    }
  }

  public static class ConfigInt extends Config<Integer> {
    public ConfigInt(String name, int defaultValue) {
      super(name, defaultValue);

      // Make sure that it shows up in the Preferences
      Preferences.initInt(name, defaultValue);
    }

    @Override
    public Integer getValue() {
      return Preferences.getInt(name, defaultValue);
    }

    public boolean isTrue() {
      return getValue() != 0;
    }

    public boolean isFalse() {
      return (!isTrue());
    }
  }

  public static class ConfigBoolean extends Config<Boolean> {
    public ConfigBoolean(String name, boolean defaultValue) {
      super(name, defaultValue);

      Preferences.initBoolean(name, defaultValue);
    }

    public Boolean getValue() {
      return Preferences.getBoolean(name, defaultValue);
    }
  }
}
