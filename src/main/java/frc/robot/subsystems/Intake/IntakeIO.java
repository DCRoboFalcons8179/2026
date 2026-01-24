// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

public interface IntakeIO {

  public static class IntakeInputs {
    public double current = 0.0;
    public double encoderPosition = 0.0;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setFeederVelocity(double velocity) {}

  public default void stop() {}
}
