// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.turret.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTurretPose extends InstantCommand {
  Turret turret;
  double desiredTurretPose;

  public SetTurretPose(Turret turret, double desiredTurretPose) {
    this.turret = turret;
    this.desiredTurretPose = desiredTurretPose;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setTurretPose(desiredTurretPose);
  }
}
