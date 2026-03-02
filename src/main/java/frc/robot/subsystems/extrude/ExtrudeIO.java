// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extrude;

public interface ExtrudeIO {

  public static class ExtrudeInputs {
    public double current = 0.0;
    public double encoderPosition = 0.0;
    public double appliedVoltage = 0.0;
    public double targetPosition = 0.0;
  }

  public default void updateInputs(ExtrudeInputs inputs) {}

  public default void setExtruderPosition(double position) {}

  public default void stop() {}
}
