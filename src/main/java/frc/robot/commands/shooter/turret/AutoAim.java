// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAim extends Command {
  private final Drive drive;
  private final Turret turret;
  private final PIDController pidController;

  /** Creates a new AutoAim. */
  public AutoAim(Drive drive, Turret turret) {
    this.drive = drive;
    this.turret = turret;
    pidController =
        new PIDController(TurretConstants.AIM_KP, TurretConstants.AIM_KI, TurretConstants.AIM_KD);
    pidController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();

    Translation2d delta =
        FieldConstants.getTargetData(FieldConstants.HUB_POSITION).minus(robotPose.getTranslation().plus(TurretConstants.TURRET_POSE));
        
    // Use Rotation2d subtraction which properly wraps the angle to [-180, 180]
    Rotation2d fieldAngle = delta.getAngle();
    Rotation2d turretAngle = fieldAngle.minus(robotPose.getRotation());

    // Clamp to the turret's physical range of [-90, 90] degrees
    double angleDegrees = MathUtil.clamp(turretAngle.getDegrees(), -90.0, 90.0);

    SmartDashboard.putNumber("Turret Angle Desired", angleDegrees);

    turret.setTurretPose(-angleDegrees / 30);
    turret.moveTurret();
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
