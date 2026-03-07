package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class TurretConstants {
  /// ID of the turret motor
  public static final int TURRET_ID = 3;

  /// The mode for the turret motor when inactive
  public static final NeutralModeValue TURRET_NEUTRAL_MODE = NeutralModeValue.Brake;

  /// KP for moving to pos
  public static final double TURRET_KP = 5;
  /// KI for moving to pos
  public static final double TURRET_KI = 0.001;
  /// KD for moving to pos
  public static final double TURRET_KD = 0;

  public static final double MAX_MOTOR_ROT = 3;
  public static final double MIN_MOTOR_ROT = -3;

  /// KP for auto aim
  public static final double AIM_KP = 0;
  /// KI for auto aim
  public static final double AIM_KI = 0;
  /// KD for auto aim
  public static final double AIM_KD = 0;

  public static final double GEAR_RATIO = 10 / 1.0;
}
