// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
  // Motor
  protected final TalonFX feeder = new TalonFX(Constants.Intake.FEEDER_ID);

  private final double feederGearRatio = Constants.Intake.GEAR_RATIO;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public IntakeIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    feeder.getConfigurator().apply(Constants.Intake.FEEDER_CURRENT_LIMIT);
    feeder.setNeutralMode(Constants.Intake.FEEDER_NEUTRAL_MODE);

    // Configure feedforward gains for velocity control
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.0; // Static friction feedforward (volts)
    slot0Configs.kV = 0.12; // Velocity feedforward (volts per rotation per second)
    slot0Configs.kA = 0.0; // Acceleration feedforward (volts per rotation per second^2)
    slot0Configs.kP = 0.2; // Proportional gain
    slot0Configs.kI = 0.0; // Integral gain
    slot0Configs.kD = 0.0; // Derivative gain
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
  public void updateInputs(IntakeInputs inputs) {
    inputs.current = feeder.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = feeder.getPosition().getValueAsDouble();
    inputs.mechanismRotationsPerSecond = feeder.getVelocity().getValueAsDouble() * feederGearRatio;
  }
}
