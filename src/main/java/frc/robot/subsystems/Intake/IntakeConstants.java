package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  // ID for feeder
  public static final int ID = 6;

  public static final double GEAR_RATIO = 1.0 / 3.0;

  // motor velocity
  public static final double SPEED_IN = -18.5;
  public static final double SPEED_OUT = 18.5;

  // Neutral Mode for feeder
  public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

  public static final CurrentLimitsConfigs CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

  public static final double KP = 0.2; // Proportional gain
  public static final double KI = 0.0; // Integral gain
  public static final double KD = 0.0; // Derivative gain
  public static final double KV = 0.12; // Velocity feedforward (volts per rotation per second)
  public static final double KA = 0.0; // Acceleration feedforward (volts per rotation per second^2)
}
