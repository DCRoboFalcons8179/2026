// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Intake.IntakeInputsAutoLogged;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double current = 0.0;
    public double appliedVoltage = 0.0;
    public double motorRotationsPerSecond = 0.0;
    public double mechanismRotationsPerSecond = 0.0;
    public double desiredMotorRPS = 0;
    public Intake.State state = Intake.State.UNDETERMINED;
  }

  public default void updateInputs(IntakeInputsAutoLogged inputs) {}

  public default void setFeederVelocity(double velocity) {}

  public default void stop() {}
}
