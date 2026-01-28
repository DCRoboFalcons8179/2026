// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shoot.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SMF.StateMachine;
import frc.robot.subsystems.shoot.turret.TurretIO.TurretInputs;

public class Turret extends StateMachine<Turret.State> {
  private final TurretIO io;

  private final TurretInputs inputs = new TurretInputs();

  private double desiredTurretPose = 0;

  public Turret(TurretIO io) {
    super("Turret", State.UNDETERMINED, State.class);
    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands();
    registerStateTransitions();
  }

  public void registerStateCommands() {

    registerStateCommand(State.LOCKED, new InstantCommand(io::stop));

    registerStateCommand(
        State.UNLOCKED,
        new SequentialCommandGroup(new InstantCommand(() -> io.moveTurret(desiredTurretPose))));
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
    io.updateInputs(inputs);
    SmartDashboard.putString("Turret State", getState().toString());
  }

  public void setTurretPose(double desiredTurretPose) {
    this.desiredTurretPose = desiredTurretPose;
  }

  public void moveTurret() {
    io.moveTurret(desiredTurretPose);
  }

  public enum State {
    // states
    UNDETERMINED,
    LOCKED,
    UNLOCKED,
    AIM
  }
}
