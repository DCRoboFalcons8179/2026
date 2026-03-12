package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface ShooterIO {

  @AutoLog
  public static class ShooterInputs {
    public double current = 0.0;
    public double appliedVoltage = 0.0;
    public double mainVelocity = 0.0;
    public double mainTargetVelocity = 0;
    public double followerVelocity = 0;
    public double followerTargetVelocity = 0;
    public Shooter.State state = Shooter.State.UNDETERMINED;
  }

  public default void setShooterTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void updateInputs(ShooterInputsAutoLogged inputs) {
    Logger.processInputs("Shooter", inputs);
  }

  public default void setPIDControl() {}

  public default boolean isCharged() {
    return false;
  }

  public default void tiltShooter(double position) {}

  public default double getVelocity() {
    return 0;
  }

  public default void beaterBarReverse(double velocity) {}
}
