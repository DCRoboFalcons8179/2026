// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hanger;

import static frc.robot.Constants.Hanger.maximum_height;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SMF.StateMachine;
import frc.robot.subsystems.Hanger.HangerIO.HangerInputs;

public class Hanger extends StateMachine<Hanger.State> {
  private final HangerIO io;

  private final HangerInputs inputs = new HangerInputs();

  public Hanger(HangerIO io) {
    super("Hanger", State.UNDETERMINED, State.class);
    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands();
    registerStateTransitions();
  }

  public void registerStateCommands() {

    // default command for idle is to stop the motor
    registerStateCommand(State.IDLE, new InstantCommand(io::stop));

    // extend will move motor to maximum height, then hold it there until a new command is given
    // this is used for readying the hanger for use, and for lowering the robot down after hanging
    registerStateCommand(
        State.EXTEND,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setPIDControl(maximum_height)),
            new WaitCommand(0.25),
            new WaitUntilCommand(() -> io.isMaxHeight()),
            new WaitCommand(0.1),
            transitionCommand(State.HOLD)));

    // retract moves motor to minimum height, and sets motor to coast, so that power can be
    // preserved
    // used for stowing the hanger when not in use
    registerStateCommand(
        State.RETRACT,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setPIDControl(0.0)),
            new WaitCommand(0.25),
            new WaitUntilCommand(() -> io.isMinHeight()),
            new WaitCommand(0.1),
            transitionCommand(State.IDLE)));

    // pull will move motor to minimum height and brake the motors
    // used for hanging
    registerStateCommand(
        State.PULL,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setPIDControl(0.0)),
            new WaitCommand(0.25),
            new WaitUntilCommand(() -> io.isMinHeight()),
            new WaitCommand(0.1),
            transitionCommand(State.HOLD)));
  }

  public void registerStateTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.HOLD);
    addOmniTransition(State.EXTEND);
    addOmniTransition(State.RETRACT);
    addOmniTransition(State.PULL);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    SmartDashboard.putString("Hanger State", getState().toString());
  }

  public enum State {
    // states
    UNDETERMINED,
    IDLE,
    EXTEND,
    RETRACT,
    PULL,
    HOLD
  }
}
