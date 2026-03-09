// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
  // Motor
  protected final TalonFX feeder = new TalonFX(IntakeConstants.FEEDER_ID);

  private final double feederGearRatio = IntakeConstants.GEAR_RATIO;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public IntakeIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    feeder.getConfigurator().apply(IntakeConstants.FEEDER_CURRENT_LIMIT);
    feeder.setNeutralMode(IntakeConstants.FEEDER_NEUTRAL_MODE);

    // Configure feedforward gains for velocity control
    Slot0Configs slot0Configs = new Slot0Configs();

    feeder.getConfigurator().apply(slot0Configs);
  }

  public void setFeederVelocity(double mechanismRotationsPerSecond) {
    double motorRotationsPerSecond = mechanismRotationsPerSecond / feederGearRatio;
    feeder.setControl(
        velocityRequest
            .withVelocity(motorRotationsPerSecond)
            .withAcceleration(40.0)
            .withEnableFOC(false));
  }

  public void stop() {
    feeder.set(0);
    feeder.stopMotor();
  }

  @Override
  public void updateInputs(IntakeInputsAutoLogged inputs) {
    inputs.current = feeder.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = feeder.getMotorVoltage().getValueAsDouble();
    inputs.motorRotationsPerSecond = feeder.getVelocity().getValueAsDouble();
    inputs.mechanismRotationsPerSecond = feeder.getVelocity().getValueAsDouble() * feederGearRatio;

    Logger.processInputs("Intake", inputs);
  }
}
