// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakes;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Intake.IntakeInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
  // Motor
  protected final TalonFX intake = new TalonFX(IntakeConstants.ID);

  private final double feederGearRatio = IntakeConstants.GEAR_RATIO;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private double desiredMotorRPS = 0;

  public IntakeIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    intake.getConfigurator().apply(IntakeConstants.CURRENT_LIMIT);
    intake.setNeutralMode(IntakeConstants.NEUTRAL_MODE);

    // Configure feedforward gains for velocity control
    Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKP(IntakeConstants.KP)
            .withKI(IntakeConstants.KI)
            .withKD(IntakeConstants.KD)
            .withKV(IntakeConstants.KV)
            .withKA(IntakeConstants.KA);

    intake.getConfigurator().apply(slot0Configs);
  }

  public void setFeederVelocity(double mechanismRotationsPerSecond) {
    double motorRotationsPerSecond = mechanismRotationsPerSecond / IntakeConstants.GEAR_RATIO;

    desiredMotorRPS = motorRotationsPerSecond;

    intake.setControl(
        velocityRequest
            .withVelocity(motorRotationsPerSecond)
            .withAcceleration(40.0)
            .withEnableFOC(false));
  }

  public void stop() {
    intake.set(0);
    intake.stopMotor();
  }

  @Override
  public void updateInputs(IntakeInputsAutoLogged inputs) {
    inputs.current = intake.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = intake.getMotorVoltage().getValueAsDouble();
    inputs.motorRotationsPerSecond = intake.getVelocity().getValueAsDouble();
    inputs.mechanismRotationsPerSecond = intake.getVelocity().getValueAsDouble() * feederGearRatio;
    inputs.desiredMotorRPS = desiredMotorRPS;

    Logger.processInputs("Intake", inputs);
  }
}
