package frc.robot.subsystems.extrude;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ExtrudeConstants {
  // ID for extruder
  public static final int ID = 7;
  public static final int FOLLOWER_ID = 16;

  // Neutral Mode for Extruder
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

  public static final CurrentLimitsConfigs CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(2).withSupplyCurrentLimitEnable(true);

  public static final double IN_POSITION = -10;
  public static final double OUT_POSITION = -33;
  public static final double MANUAL_DELTA = 3;
  public static final double MIN_POS = -34;
  public static final double MAX_POS = 0;

  public static final InvertedValue INVERT = InvertedValue.CounterClockwise_Positive;
  // Error thresh hold (current just a placeholder number before being tested)
  public static final double ERROR_THRESH_HOLD = .2;

  public static final double GEAR_RATIO = 1 / 3;

  // PID values for extruder - These need tuning
  public static final double KP = 24;
  public static final double KI = 0.3;
  public static final double KD = 0.2;

  // Motion profiling constraints to control speed
  // Maximum velocity in rotations per second (adjust lower to slow down)
  public static final double MAX_VELOCITY = 60.0;
  // Maximum acceleration in rotations per second squared
  public static final double MAX_ACCELERATION = 30.0; // Smooth acceleration
  // Jerk control for even smoother motion (rotations per second cubed)
  public static final double JERK = 40.0; // Optional smoothing

  // Agitate state - oscillates using velocity control
  // Dynamic inward limit: reverses when current draw nears the supply limit
  public static final double AGITATE_OUT_POS = OUT_POSITION;
  public static final double AGITATE_IN_POS = IN_POSITION;
  public static final double AGITATE_VELOCITY = 15.0; // rps - speed for agitation (mechanism)
  public static final double AGITATE_SLOW_VELOCITY = 5.0; // rps - slower speed near the in limit
  public static final double AGITATE_SLOW_DISTANCE =
      5.0; // rotations before the in pos to start slowing
  public static final double AGITATE_CURRENT_THRESHOLD = CURRENT_LIMIT.SupplyCurrentLimit;
}
