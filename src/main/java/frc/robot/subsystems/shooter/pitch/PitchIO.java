package frc.robot.subsystems.shooter.pitch;

import org.littletonrobotics.junction.AutoLog;

public interface PitchIO {

  @AutoLog
  public static class PitchInputs {
    public double current = 0.0;
    public double encoderPosition = 0.0;
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double targetPosition = 0.0;
    public boolean atTarget = false;
  }

  public default void stop() {}

  public default void updateInputs(PitchInputs inputs) {}

  public default void setPIDControl() {}

  /**
   * Moves the pitch to a desired spot
   *
   * @param position - Position to move the pitch to
   */
  public default void movePitch(double position) {}
}
