// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extrude;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ExtrudeIOReal implements ExtrudeIO {

  // PID control
  private PIDController ExtruderPID =
      new PIDController(
          Constants.Extruder.EXTRUDER_KP,
          Constants.Extruder.EXTRUDER_KI,
          Constants.Extruder.EXTRUDER_KD);

  // Motor
  protected final TalonFXS extruder = new TalonFXS(Constants.Extruder.EXTRUDER_ID);

  // Control request for position control
  private final PositionVoltage positionControl = new PositionVoltage(0);

  public ExtrudeIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    // gives a current limit
    extruder.getConfigurator().apply(Constants.Extruder.EXTRUDER_CURRENT_LIMIT);
    //sets desirded neutral state
    extruder.setNeutralMode(Constants.Extruder.EXTRUDER_NEUTRAL_MODE);
    // Reset encoder position to 0 on startup
    extruder.setPosition(0);
  }

  @Override
  public void updateInputs(ExtrudeInputs inputs) {
    inputs.encoderPosition = extruder.getPosition().getValueAsDouble();
    inputs.current = extruder.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setExtruderPosition(double position) {
    extruder.setControl(positionControl.withPosition(position));
  }

  @Override
  public void stop() {
    extruder.stopMotor();
  }

  public void setPIDControl() {
    ExtruderPID.setSetpoint(2.0);
    extruder.setVoltage(ExtruderPID.calculate(extruder.getTorqueCurrent().getValueAsDouble()));
  }
}
