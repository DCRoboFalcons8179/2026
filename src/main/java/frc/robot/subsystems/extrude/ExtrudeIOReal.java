// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extrude;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ExtrudeIOReal implements ExtrudeIO {

  // PID control
  private PIDController ExtruderPID = new PIDController(0.25, 0, 0);

  // Motor
  protected final TalonFXS extruder = new TalonFXS(Constants.Intake.EXTRUDER_ID);

  public ExtrudeIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    extruder.setNeutralMode(Constants.Intake.EXTRUDER_NEUTRAL_MODE);
  }

  public void setPIDControl() {
    ExtruderPID.setSetpoint(2.0);
    extruder.setVoltage(ExtruderPID.calculate(extruder.getTorqueCurrent().getValueAsDouble()));
  }
}
