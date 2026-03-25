// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShootVelocity extends Command {
  private final Shooter shooter;
  private double launchVelocity = ShooterConstants.OUTPUT_SPEED;
  private Supplier<Pose2d> poseSupplier;

  /** Creates a new AutoShootVelocity. */
  public AutoShootVelocity(Shooter shooter, Supplier<Pose2d> poseSupplier) {
    this.shooter = shooter;
    this.poseSupplier = poseSupplier;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double distance = vision.getTargetDistance(1).getDistance(Translation2d.kZero);

    // int targetID = vision.getBestTagId(1);

    // double[] badBlueIDs = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    // double[] badRedIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    // double[] trenchIDs = {1, 12, 6, 7, 22, 23, 17, 28};

    // if (DriverStation.getAlliance().get() == Alliance.Red) {
    //   for (double id : badBlueIDs) {
    //     if (id == targetID) {
    //       launchVelocity = ShooterConstants.OUTPUT_SPEED;
    //       shooter.setVelocity(launchVelocity);
    //       return;
    //     }
    //   }
    // } else {
    //   for (double id : badRedIDs) {
    //     if (id == targetID) {
    //       launchVelocity = ShooterConstants.OUTPUT_SPEED;
    //       shooter.setVelocity(launchVelocity);
    //       return;
    //     }
    //   }
    Pose2d robotPose = poseSupplier.get();

    double distance =
        FieldConstants.getTargetData(FieldConstants.HUB_POSITION)
            .getDistance(robotPose.getTranslation());

    // Remove all trench tags
    //   for (double id : trenchIDs) {
    //     if (id == targetID) {
    //       launchVelocity = ShooterConstants.OUTPUT_SPEED;
    //       shooter.setVelocity(launchVelocity);
    //       return;
    //     }
    //   }

    SmartDashboard.putNumber("Distance to Target", distance);

    launchVelocity = (7.725 * distance) + 28.723 - 1;
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
