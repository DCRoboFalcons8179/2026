package frc.robot.subsystems.bellyBeaterBar;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class BellyBeaterBarConstants {
  /// Belly Beater Bar Motor ID
  public static final int ID = 3;

  public static final double IN_SPEED = 1;
  public static final double OUT_SPEED = -1;

  public static final InvertedValue INVERT = InvertedValue.Clockwise_Positive;

  public static final NeutralModeValue NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;
}
