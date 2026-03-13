// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShootVelocity extends Command {
  private final Vision vision;
  private final Shooter shooter;
  private double launchVelocity = 0;

  /** Creates a new AutoShootVelocity. */
  public AutoShootVelocity(Shooter shooter, Vision vision) {
    this.shooter = shooter;
    this.vision = vision;
    addRequirements(shooter, vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = vision.getTargetDistance(1).getDistance(Translation2d.kZero);

    SmartDashboard.putNumber("Distance to Target", distance);

    if (distance == 0) {
      launchVelocity = ShooterConstants.OUTPUT_SPEED;
    } else {
      // Calculate the launch velocity based on the distance to the target
      launchVelocity = (7.642 * distance) + 28.723 - 0;
    }

    shooter.setVelocity(launchVelocity);
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
