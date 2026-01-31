// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.turret;

import static frc.robot.Constants.C_Shooter.C_Turret.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAim extends Command {
  private final Vision vision;
  private final Turret turret;
  private final PIDController pidController;

  /** Creates a new AutoAim. */
  public AutoAim(Vision vision, Turret turret) {
    this.vision = vision;
    this.turret = turret;
    pidController = new PIDController(AIM_KP, AIM_KI, AIM_KD);
    pidController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(vision, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double omegaPercent = vision.getOmegaPercentOut(pidController);

    turret.aimPercentOut(omegaPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
