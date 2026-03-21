// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.math.DriveByCalcs;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAim extends Command {
  private final Turret turret;
  private final PIDController pidController;

  /** Creates a new AutoAim. */
  public AutoAim(Turret turret) {
    this.turret = turret;
    pidController = new PIDController(TurretConstants.AIM_KP, TurretConstants.AIM_KI, TurretConstants.AIM_KD);
    pidController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turret angle in degrees
    double turretAngle = Math.toDegrees(Math.atan(DriveByCalcs.getNumerator() / DriveByCalcs.getDenominator()));

    turret.setTurretDegrees(turretAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
