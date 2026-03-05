// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SMF.StateMachine;
import frc.robot.commands.shooter.turret.TurretToPose;
import frc.robot.subsystems.vision.Vision;

public class Turret extends StateMachine<Turret.State> {
  private final TurretIO io;
  private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();

  private final Vision vision;

  private double desiredTurretPose = 0;

  public Turret(TurretIO io, Vision vision) {
    super("Turret", State.UNDETERMINED, State.class);
    this.io = io;
    this.vision = vision;

    io.updateInputs(inputs);

    registerStateCommands();
    registerStateTransitions();
  }

  public void registerStateCommands() {

    registerStateCommand(
        State.LOCKED,
        new SequentialCommandGroup(
            new InstantCommand(() -> this.desiredTurretPose = io.getTurretPosition()),
            new TurretToPose(this)));

    registerStateCommand(State.UNLOCKED, new TurretToPose(this));

    // Has the turret aim when the aim state is set
    // registerStateCommand(State.AIM, new AutoAim(vision, this));
  }

  public void registerStateTransitions() {
    addOmniTransition(State.LOCKED);
    addOmniTransition(State.UNLOCKED);
    addOmniTransition(State.AIM);
  }

  @Override
  protected void determineSelf() {
    setState(State.LOCKED);
  }

  @Override
  protected void update() {
    inputs.state = this.getState();
    io.updateInputs(inputs);
  }

  public void setTurretPose(double desiredTurretPose) {
    if (getState() != State.LOCKED) {
      this.desiredTurretPose = desiredTurretPose;
    }
  }

  public void moveTurret() {
    io.moveTurret(desiredTurretPose);
  }

  /**
   * Aim the turret with percent out
   *
   * @param omegaPercent - Percent out to give to the turret
   */
  public void aimPercentOut(double omegaPercent) {
    io.moveTurretPO(omegaPercent);
  }

  public enum State {
    // states
    UNDETERMINED,
    LOCKED,
    UNLOCKED,
    AIM
  }
}
