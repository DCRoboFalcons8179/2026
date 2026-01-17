// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIOReal implements IntakeIO {

  //PID control
  private PIDController pid = new PIDController(0, 0, 0);

  protected final TalonFXS intake = new TalonFXS(0);
  protected final TalonFXS hinge = new TalonFXS(0);
  public IntakeIOReal() {

  }


}
