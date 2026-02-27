// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.extrude.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveExtrude extends Command {
  /** Creates a new MoveExtrude. */
  private Extrude extrude;

  private double targetPosition;

  public MoveExtrude(Extrude extrude) {
    this.extrude = extrude;
    addRequirements(extrude);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the target position when the command starts
    targetPosition = extrude.getDesiredPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Nothing needed here - the motor is already moving via setExtruderPosition
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Transition to IDLE when movement is complete
    if (!interrupted) {
      extrude.transitionCommand(Extrude.State.IDLE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if we're within the error threshold of the target
    double currentPos = extrude.getPos();
    return Math.abs(targetPosition - currentPos) < Constants.Extruder.EXTRUDER_ERROR_THRESH_HOLD;
  }
}
