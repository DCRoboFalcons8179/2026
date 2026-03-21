package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;

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

  public static final double NUDGE_AMOUNT = 0.33 / 2;

  /// KP for auto aim
  public static final double AIM_KP = 0;
  /// KI for auto aim
  public static final double AIM_KI = 0;
  /// KD for auto aim
  public static final double AIM_KD = 0;

  public static final double GEAR_RATIO = 10 / 1.0;

  // All in degrees

  public static final double TRENCH_ANGLE = 60;

  public static final double CORNER_ANGLE = 45;

  public static final double TOWER_ANGLE = 0;

  public static final double LEBRON_ANGLE = 90;

  public static final double ZERO_ANGLE = 0;

  // TODO: Turret pose to robot center
  public static final Translation2d TURRET_POSE = new Translation2d(0, 0.5);

  // TODO: Add actual height
  public static final double TURRET_HEIHGT = 1;

  // TODO: Set real pitch
  public static final double PITCH = 20;
}
