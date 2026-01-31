package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  public static class TurretInputs {
    public double current = 0.0;
    public double encoderPosition = 0.0;
    public double velocity = 0.0;
    public double appliedVoltage = 0.0;
    public double targetPosition = 0.0;
    public boolean atTarget = false;
  }

  public default void stop() {}

  public default void updateInputs(TurretInputs inputs) {}

  public default void setPIDControl() {}

  /**
   * Moves the turret to a desired spot
   * @param position - Position to move the turret to
   */
  public default void moveTurret(double position) {}

  /** 
   * Moves the turret using percent out
   * @param omegaPercent - Percent out to give to the turret */
  public default void moveTurretPO(double omegaPercent) {}
}
