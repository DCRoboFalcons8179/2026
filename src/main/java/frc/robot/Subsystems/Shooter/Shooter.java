// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final TalonFX leadShooter;

  /** Creates a new Shooter. */
  public Shooter() {
    leadShooter = new TalonFX(Constants.Shooter.leadShooterID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
