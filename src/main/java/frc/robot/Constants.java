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
    public static class ShooterSubsystem{
      public static final int MOTOR1 = 5;
      public static final int MOTOR2 = 6;
    }
  }

  public static class PID{
    public static class Shooter{
      public static final ConfigDouble kP = new ConfigDouble("Shooter/kP", 1);
      public static final ConfigDouble kI = new ConfigDouble("Shooter/kI", 0);
      public static final ConfigDouble kD = new ConfigDouble("Shooter/kD", 0);
    }
    public static class Drive {
      public static double HEADING_CORRECTION_KP = 0.02;
      public static double HEADING_CORRECTION_KI = 0;
      public static double HEADING_CORRECTION_KD = 0.0035;
      public static double HEADING_CORRECTION_KS = 0.047;
      public static double HEADING_CORRECTION_KV = 0;
      public static double HEADING_CORRECTION_TOLERANCE = 0.05;
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
