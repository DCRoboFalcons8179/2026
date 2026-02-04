package frc.robot.subsystems.Hanger;

import org.littletonrobotics.junction.AutoLog;

public interface HangerIO {

  @AutoLog
  public static class HangerInputs {
    public double current = 0.0;
    public double encoderPosition = 0.0;
  }

  public default void setHangerTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void updateInputs(HangerInputs inputs) {}

  public default void setPIDControl(double setpoint) {}

  public default boolean isMaxHeight() {
    return false;
  }
}
