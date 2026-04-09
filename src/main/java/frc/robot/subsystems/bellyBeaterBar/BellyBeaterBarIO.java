package frc.robot.subsystems.bellyBeaterBar;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface BellyBeaterBarIO {

  @AutoLog
  public static class BellyBeaterBarInputs {
    public double current = 0.0;
    public double appliedVoltage = 0.0;
    public double percentOut = 0;
    public BellyBeaterBar.State state = BellyBeaterBar.State.UNDETERMINED;
  }

  public default void setBellyBeaterBarTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void updateInputs(BellyBeaterBarInputsAutoLogged inputs) {
    Logger.processInputs("BellyBeaterBar", inputs);
  }

  public default void inBBB() {}

  public default void outBBB() {}

  public default void stopBBB() {}
}
