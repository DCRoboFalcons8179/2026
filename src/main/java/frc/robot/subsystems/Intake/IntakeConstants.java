package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  // ID for feeder
  public static final int FEEDER_ID = 6;

  public static final double GEAR_RATIO = 1.0 / 3.0;

  // motor velocity (current just a placeholder number before being tested)
  public static final double FEEDER_SPEED_IN = -18.5;
  public static final double FEEDER_SPEED_OUT = 18.5;

  // Neutral Mode for feeder
  public static final NeutralModeValue FEEDER_NEUTRAL_MODE = NeutralModeValue.Coast;

  public static final CurrentLimitsConfigs FEEDER_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

  public static final double KP = 0.2; // Proportional gain
  public static final double KI = 0.0; // Integral gain
  public static final double KD = 0.0; // Derivative gain
  public static final double KV = 0.12; // Velocity feedforward (volts per rotation per second)
  public static final double KA = 0.0; // Acceleration feedforward (volts per rotation per second^2)
}
