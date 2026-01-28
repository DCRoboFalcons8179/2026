package frc.robot.subsystems.Shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretInputs {
    public double current = 0.0;
    public double encoderPosition = 0.0;
  }

  public default void stop() {}

  public default void updateInputs(TurretInputs inputs) {}

  public default void setPIDControl() {}

  public default void moveTurret(double position) {}
}
