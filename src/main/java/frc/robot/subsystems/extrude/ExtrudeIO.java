// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extrude;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface ExtrudeIO {

  @AutoLog
  public static class ExtrudeInputs {
    public double current = 0.0;
    public double encoderPosition = 0.0;
    public double appliedVoltage = 0.0;
    public double targetPosition = 0.0;
    public Extrude.State state = Extrude.State.UNDETERMINED;
  }

  public default void updateInputs(ExtrudeInputsAutoLogged inputs) {
    Logger.processInputs("Extrude", inputs);
  }

  public default void setExtruderPosition(double position) {}

  public default void stop() {}

  public default void addExtruderPosition(double extruderManalDelta) {}

  public default double getTargetPosition() {
    return 0;
  }

  public default double getPosition() {
    return 0;
  }
}
