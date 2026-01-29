// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {

  // PID control
  private PIDController feederPID = new PIDController(0.25, 0, 0);

  // Motor
  protected final TalonFXS feeder = new TalonFXS(Constants.Intake.FEEDER_ID);

  public IntakeIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    feeder.setNeutralMode(Constants.Intake.FEEDER_NEUTRAL_MODE);
  }

  public void setFeederTargetVelocity(double velocity) {
    feeder.set(velocity);
  }

  public void stopFeeder() {
    feeder.set(0);
    feeder.stopMotor();
  }

  public void setPIDControl() {}

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.current = feeder.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = feeder.getPosition().getValueAsDouble();
  }
}
