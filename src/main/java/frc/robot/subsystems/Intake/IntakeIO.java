// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double current = 0.0;
    public double appliedVoltage = 0.0;
    public double motorRotationsPerSecond = 0.0;
    public double mechanismRotationsPerSecond = 0.0;
    public Intake.State state = Intake.State.UNDETERMINED;
  }

  public default void updateInputs(IntakeInputsAutoLogged inputs) {}

  public default void setFeederVelocity(double velocity) {}

  public default void stop() {}
}
