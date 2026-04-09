// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SMF.StateMachine;
import frc.robot.commands.shooter.AutoShootVelocity;

import java.util.function.Supplier;

public class Shooter extends StateMachine<Shooter.State> {
  private final ShooterIO io;

  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
  private final Supplier<Pose2d> poseSupplier;

  public Shooter(ShooterIO io, Supplier<Pose2d> poseSupplier) {
    super("Shooter", State.UNDETERMINED, State.class);
    this.io = io;
    this.poseSupplier = poseSupplier;

    io.updateInputs(inputs);

    registerStateCommands();
    registerStateTransitions();
  }

  public void registerStateCommands() {
    registerStateCommand(State.IDLE, new InstantCommand(io::stop));

    registerStateCommand(
        State.CHARGE,
        new SequentialCommandGroup(
            new AutoShootVelocity(this, poseSupplier).withTimeout(0.25),
            new WaitUntilCommand(() -> io.isCharged()),
            new WaitCommand(0.1),
            new InstantCommand(() -> requestTransition(State.SHOOT))));

    registerStateCommand(State.SHOOT, new AutoShootVelocity(this, poseSupplier));

    registerStateCommand(State.REVERSE, new InstantCommand(() -> io.beaterBarReverse(-50)));
  }

  public void registerStateTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.CHARGE);
    addOmniTransition(State.SHOOT);
    addOmniTransition(State.REVERSE);
  }

  public void setVelocity(double velocity) {
    io.setShooterTargetVelocity(velocity);
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  public void empty() {
    System.out.println("EMPTY");
    io.outBBB();
  }

  public void stopBBB() {
    System.out.println("STOP");
    io.stopBBB();
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
    SHOOT,
    REVERSE
  }
}
