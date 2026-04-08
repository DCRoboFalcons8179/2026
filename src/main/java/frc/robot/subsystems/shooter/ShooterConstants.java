package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterConstants {
  /// ID of Lead Shooter
  public static final int ID = 4;
  /// boolean for the Lead Shooter's invert
  public static final InvertedValue INVERT = InvertedValue.CounterClockwise_Positive;

  /// KP for Lead Shooter
  public static final double KP = 1;
  /// KI for Lead Shooter
  public static final double KI = 0;
  /// KD for Lead Shooter
  public static final double KD = 0;
  /// KV for Shooter
  public static final double KV = 0.2;

  public static final double FOLLOWER_KP = 0.8;

  public static final double FOLLOWER_KI = 0;

  public static final double FOLLOWER_KD = 0;

  public static final double FOLLOWER_KV = 0.2;

  /// Current to limit the motors to in amps
  public static final CurrentLimitsConfigs CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

  public static final double GEAR_RATIO = 1.0 / 1.0;

  /// Velocity for the motors to set rot/s (will be calculated in later versions,
  /// setting up to get something working rn)
  /// Multiply by 1/GEAR_RATIO to convert from output shaft speed to motor speed
  public static final double OUTPUT_SPEED = 40 * 1 / GEAR_RATIO;

  /// Velocity in rot/s that shooter speed can be under by
  public static final double LOWER_ERROR_MARGIN = 1;
  /// Velocity in rot/s that shooter speed can be over by
  public static final double UPPER_ERROR_MARGIN = 15;

  /// The mode for the motors when inactive
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

  public static final NeutralModeValue FOLLOWER_NEUTRAL_MODE = NeutralModeValue.Coast;

  /// ID of feed motor
  public static final int FEED_ID = 2;

  public static final InvertedValue FEED_INVERT = InvertedValue.CounterClockwise_Positive;

  /// KP for Shoot feeder
  public static final double FEED_KP = 1.3;
  /// KI for Shoot feeder
  public static final double FEED_KI = 0;
  /// KD for Shoot feeder
  public static final double FEED_KD = 0;
  /// KV for Shoot feeder
  public static final double FEED_KV = 0.2;
  /// Current to limit the motors to in amps
  public static final CurrentLimitsConfigs FEED_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

  private static final double FEED_GEAR_RATIO = 1.0 / 5.0;

  /// Velocity for the motors to set rot/s (will be calculated in later versions,
  /// setting up to get something working rn)
  /// Multiply by 1/GEAR_RATIO to convert from output shaft speed to motor speed
  public static final double FEED_OUTPUT_SPEED = 15 * 1 / FEED_GEAR_RATIO;

  public static final double TRENCH_VELOCITY = 57;

  // 78.90931314
  public static final double LEBRON_VELOCITY = TRENCH_VELOCITY;

  public static final double TOWER_VELOCITY = 40;

  public static final double CORNER_VELOCITY = 67;

  public static final double TRENCH_ARC = 71.2962679306;

  // Belly Beater Bar
  public static final int BBB_ID = 3;

  public static final double BBB_IN_SPEED = 0.8;
  public static final double BBB_OUT_SPEED = -0.8;

  public static final InvertedValue BBB_INVERT = InvertedValue.Clockwise_Positive;
}
