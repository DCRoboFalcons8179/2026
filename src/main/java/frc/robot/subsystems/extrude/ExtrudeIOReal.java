// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extrude;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ExtrudeIOReal implements ExtrudeIO {

  // Motor
  protected final TalonFXS extruder = new TalonFXS(Constants.Extruder.EXTRUDER_ID);

  private double targetPosition = 0;

  // Control request for Motion Magic position control (smoother than basic position control)
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

  public ExtrudeIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    // Configure PID gains for the TalonFX's built-in position control
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Extruder.KP;
    slot0Configs.kI = Constants.Extruder.KI;
    slot0Configs.kD = Constants.Extruder.KD;

    extruder.getConfigurator().apply(slot0Configs);
    extruder
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(Constants.Extruder.INVERT));

    // Configure Motion Magic for smooth, controlled movement
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Extruder.EXTRUDER_MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Extruder.EXTRUDER_MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = Constants.Extruder.EXTRUDER_JERK;
    extruder.getConfigurator().apply(motionMagicConfigs);

    // gives a current limit
    extruder.getConfigurator().apply(Constants.Extruder.EXTRUDER_CURRENT_LIMIT);
    // sets desirded neutral state
    extruder.setNeutralMode(Constants.Extruder.EXTRUDER_NEUTRAL_MODE);
    // Reset encoder position to 0 on startup
    extruder.setPosition(0);
  }

  @Override
  public void updateInputs(ExtrudeInputsAutoLogged inputs) {
    inputs.encoderPosition = extruder.getPosition().getValueAsDouble();
    inputs.current = extruder.getSupplyCurrent().getValueAsDouble();
    inputs.appliedVoltage = extruder.getMotorVoltage().getValueAsDouble();
    inputs.targetPosition = targetPosition;
    Logger.processInputs("Extrude", inputs);
  }

  @Override
  public void setExtruderPosition(double position) {
    targetPosition = position;
    // Use Motion Magic for smooth, velocity-limited movement
    extruder.setControl(motionMagicControl.withPosition(position));
  }

  @Override
  public void addExtruderPosition(double Delta) {
    targetPosition += Delta;
    setExtruderPosition(targetPosition);
  }

  @Override
  public double getTargetPosition() {
    return targetPosition;
  }

  @Override
  public void stop() {
    extruder.stopMotor();
  }
}
