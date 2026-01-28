package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterInputs {
    public double current = 0.0;
    public double encoderPosition = 0.0;
  }

  public default void setShooterTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setPIDControl() {}

  public default boolean isCharged() {
    return false;
  }

  public default void moveTurret(double position) {}

  public default void tiltShooter(double position) {}
}
