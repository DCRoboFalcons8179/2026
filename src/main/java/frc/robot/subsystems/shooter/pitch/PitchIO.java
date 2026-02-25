package frc.robot.subsystems.shooter.pitch;

import org.littletonrobotics.junction.AutoLog;

public interface PitchIO {

  @AutoLog
  public static class PitchInputs {
    public double encoderPosition = 0.0;
    public double pitch = 0.0;
  }

  public default void setShooterTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void updateInputs(PitchInputs inputs) {}

  public default void setPIDControl() {}

  public default void tiltShooter(double position) {}
}
