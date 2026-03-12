package frc.robot.subsystems.extrude;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ExtrudeConstants {
  // ID for extruder
  public static final int ID = 7;

  // Neutral Mode for Extruder
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

  public static final CurrentLimitsConfigs CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(5).withSupplyCurrentLimitEnable(true);

  public static final double IN_POSITION = 0;
  public static final double OUT_POSITION = -32;
  public static final double MANUAL_DELTA = 3;

  public static final InvertedValue INVERT = InvertedValue.CounterClockwise_Positive;
  // Error thresh hold (current just a placeholder number before being tested)
  public static final double ERROR_THRESH_HOLD = .2;

  // PID values for extruder - These need tuning
  public static final double KP = 24;
  public static final double KI = 0.3;
  public static final double KD = 0.2;

  // Motion profiling constraints to control speed
  // Maximum velocity in rotations per second (adjust lower to slow down)
  public static final double MAX_VELOCITY = 48.0; // 24 rotations/sec - adjust as needed
  // Maximum acceleration in rotations per second squared
  public static final double MAX_ACCELERATION = 21.0; // Smooth acceleration
  // Jerk control for even smoother motion (rotations per second cubed)
  public static final double JERK = 40.0; // Optional smoothing
}
