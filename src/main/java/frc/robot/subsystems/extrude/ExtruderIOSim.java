// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extrude;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ExtruderIOSim implements ExtrudeIO {

  private final DCMotorSim extruderSim;

  // PID controller for motion magic simulation
  private final PIDController positionController;

  // Applied values
  private double appliedVoltage = 0.0;
  private double targetPosition = 0.0;

  private static final double EXTRUDER_GEARING = 100.0; // Adjust based on your gearing ratio
  private static final double EXTRUDER_MOI = 0.001; // kg*m^2, adjust for your turret mass

  public ExtruderIOSim() {

    extruderSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1),
                EXTRUDER_MOI,
                EXTRUDER_GEARING),
            DCMotor.getKrakenX44(1));

    positionController =
      new PIDController(Constants.Extruder.EXTRUDER_KP, Constants.Extruder.EXTRUDER_KI, Constants.Extruder.EXTRUDER_KD);
    
    setPIDControl();
  }

  public void setPIDControl() {
    positionController.setPID(Constants.Extruder.EXTRUDER_KP, Constants.Extruder.EXTRUDER_KI, Constants.Extruder.EXTRUDER_KD);
  }

   @Override
  public void stop() {
    appliedVoltage = 0.0;
    positionController.reset();
  }

   @Override
  public void updateInputs(ExtrudeInputs inputs) {

    // Update simulation with applied voltage
    extruderSim.setInputVoltage(appliedVoltage);

    // Update simulation (assuming 20ms loop time)
    extruderSim.update(0.02);

    inputs.current = extruderSim.getCurrentDrawAmps();
    inputs.appliedVoltage = appliedVoltage;
    inputs.targetPosition = targetPosition;
    }
}
