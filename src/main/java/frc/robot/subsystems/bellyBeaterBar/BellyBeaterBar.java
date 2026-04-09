// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bellyBeaterBar;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SMF.StateMachine;

public class BellyBeaterBar extends StateMachine<BellyBeaterBar.State> {
  protected final BellyBeaterBarIO io;

  protected final BellyBeaterBarInputsAutoLogged inputs = new BellyBeaterBarInputsAutoLogged();
  
  public BellyBeaterBar(BellyBeaterBarIO io) {
    super("BellyBeaterBar", State.UNDETERMINED, State.class);
    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands();
    registerStateTransitions();
  }

  public void registerStateCommands() {
    registerStateCommand(State.IDLE, new InstantCommand(io::stop));

    registerStateCommand(State.IN, new InstantCommand(() -> inBBB()));

    registerStateCommand(State.OUT, new InstantCommand(() -> outBBB()));
  }

  public void registerStateTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.IN);
    addOmniTransition(State.OUT);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  protected void inBBB() {
    io.inBBB();
  }

  protected void outBBB() {
    io.outBBB();
  }

  @Override
  protected void update() {
    inputs.state = this.getState();
    io.updateInputs(inputs);
  }

  public enum State {
    // states
    UNDETERMINED,
    IDLE,
    IN,
    OUT
  }
}
