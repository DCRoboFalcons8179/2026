// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.pitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SMF.StateMachine;
import frc.robot.commands.shooter.pitch.PitchToPose;
import frc.robot.subsystems.shooter.pitch.PitchIO.PitchInputs;

public class Pitch extends StateMachine<Pitch.State> {
  private final PitchIO io;
  private final PitchInputs inputs = new PitchInputs();

  private double desiredPitchPose = 0;

  public Pitch(PitchIO io) {
    super("Pitch", State.UNDETERMINED, State.class);
    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands();
    registerStateTransitions();
  }

  public void registerStateCommands() {
    registerStateCommand(
        State.LOCKED,
        new SequentialCommandGroup(
            new InstantCommand(() -> this.desiredPitchPose = io.getPitchPosition()),
            new PitchToPose(this)));

    registerStateCommand(State.UNLOCKED, new PitchToPose(this));
  }

  public void registerStateTransitions() {
    addOmniTransition(State.LOCKED);
    addOmniTransition(State.UNLOCKED);
  }

  @Override
  protected void determineSelf() {
    setState(State.LOCKED);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    SmartDashboard.putString("Pitch State", getState().toString());
    SmartDashboard.putNumber("Pitch Target Position", desiredPitchPose);
    SmartDashboard.putNumber("Pitch Position", io.getPitchPosition());
  }

  public void setPitchPose(double desiredPitchPose) {
    if (getState() != State.LOCKED) {
      this.desiredPitchPose = desiredPitchPose;
    }
  }

  public void movePitch() {
    io.movePitch(desiredPitchPose);
  }

  public enum State {
    // states
    UNDETERMINED,
    LOCKED,
    UNLOCKED
  }
}
