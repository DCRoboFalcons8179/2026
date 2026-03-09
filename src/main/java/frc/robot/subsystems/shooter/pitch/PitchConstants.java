package frc.robot.subsystems.shooter.pitch;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class PitchConstants {
  /// ID of the y-axis controlling motor
  public static final int PITCH_ID = 5;

  public static final double PITCH_KP = 1;
  public static final double PITCH_KI = 0;
  public static final double PITCH_KD = 0;

  public static final NeutralModeValue PITCH_NEUTRAL_MODE = NeutralModeValue.Brake;
}
