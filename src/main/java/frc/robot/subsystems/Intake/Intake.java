// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SMF.StateMachine;

public class Intake extends StateMachine<Intake.State> {
  /** Creates a new Intake. */
  public final IntakeIO io;

  public final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  public Intake(IntakeIO io) {
    super("Intake", State.UNDETERMINED, State.class);
    this.io = io;

    io.updateInputs(inputs);

    registerStateCommand();
    registerStateTransition();
  }

  public void registerStateCommand() {
    registerStateCommand(State.IDLE, new InstantCommand(io::stop));
    registerStateCommand(
        State.FEED_IN, new InstantCommand(() -> io.setFeederVelocity(IntakeConstants.SPEED_IN)));
    registerStateCommand(
        State.FEED_OUT, new InstantCommand(() -> io.setFeederVelocity(IntakeConstants.SPEED_OUT)));
  }

  public void registerStateTransition() {
    addOmniTransition(State.UNDETERMINED);
    addOmniTransition(State.IDLE);
    addOmniTransition(State.FEED_IN);
    addOmniTransition(State.FEED_OUT);
  }

  @Override
  public void determineSelf() {
    setState(State.IDLE);
  }

  @Override
  protected void update() {
    inputs.state = this.getState();
    io.updateInputs(inputs);
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    FEED_IN,
    FEED_OUT,
  }
}
