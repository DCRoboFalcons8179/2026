// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Intake.IntakeInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim extruderSim;

  // PID controller for motion magic simulation
  private final PIDController positionController;

  // Applied values
  private double appliedVoltage = 0.0;

  private static final double GEARING = 100.0; // Adjust based on your gearing ratio
  private static final double MOI = 0.001; // kg*m^2, adjust for your turret mass

  public IntakeIOSim() {

    extruderSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), MOI, GEARING),
            DCMotor.getKrakenX44(1));

    positionController =
        new PIDController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD);

    setPIDControl();
  }

  public void setPIDControl() {
    positionController.setPID(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    positionController.reset();
  }

  @Override
  public void updateInputs(IntakeInputsAutoLogged inputs) {

    // Update simulation with applied voltage
    extruderSim.setInputVoltage(appliedVoltage);

    // Update simulation (assuming 20ms loop time)
    extruderSim.update(0.02);

    inputs.current = extruderSim.getCurrentDrawAmps();
    inputs.appliedVoltage = extruderSim.getInputVoltage();
    inputs.motorRotationsPerSecond = extruderSim.getAngularVelocityRadPerSec();
    inputs.motorRotationsPerSecond =
        extruderSim.getAngularVelocityRadPerSec() * IntakeConstants.GEAR_RATIO;

    Logger.processInputs("Intake", inputs);
  }
}
