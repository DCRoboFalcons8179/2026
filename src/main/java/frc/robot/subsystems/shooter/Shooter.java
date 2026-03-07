// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SMF.StateMachine;

public class Shooter extends StateMachine<Shooter.State> {
  private final ShooterIO io;

  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  public Shooter(ShooterIO io) {
    super("Shooter", State.UNDETERMINED, State.class);
    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands();
    registerStateTransitions();
  }

  public void registerStateCommands() {

    registerStateCommand(State.IDLE, new InstantCommand(io::stop));

    registerStateCommand(
        State.CHARGE,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setShooterTargetVelocity(ShooterConstants.OUTPUT_SPEED)),
            new WaitCommand(0.25),
            new WaitUntilCommand(() -> io.isCharged()),
            new WaitCommand(0.1),
            new InstantCommand(() -> requestTransition(State.SHOOT))));

    registerStateCommand(
        State.SHOOT,
        new InstantCommand(() -> io.setShooterTargetVelocity(ShooterConstants.OUTPUT_SPEED)));
  }

  public void registerStateTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.CHARGE);
    addOmniTransition(State.SHOOT);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
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
    CHARGE,
    SHOOT
  }
}
