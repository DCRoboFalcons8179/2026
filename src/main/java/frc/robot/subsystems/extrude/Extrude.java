// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extrude;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SMF.StateMachine;
import frc.robot.commands.MoveExtrude;
import frc.robot.subsystems.intake.Intake;

public class Extrude extends StateMachine<Extrude.State> {
  /** Creates a new Intake. */
  public final ExtrudeIO io;

  public final ExtrudeInputsAutoLogged inputs = new ExtrudeInputsAutoLogged();

  public double desiredPos;

  private final Intake intake;

  /// Tracks if we already used auto intake on
  private boolean autoIntake = false;

  /// Tracks the current agitate direction (true = moving out, false = moving in)
  private boolean agitateOut = true;

  public Extrude(ExtrudeIO io, Intake intake) {
    super("Extruder", State.UNDETERMINED, State.class);
    this.io = io;
    this.intake = intake;

    io.updateInputs(inputs);

    registerStateCommand();
    registerStateTransition();
  }

  public void registerStateCommand() {
    registerStateCommand(State.IDLE, new InstantCommand(io::stop));
    registerStateCommand(
        State.EXTRUDE_IN,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setExtruderPosition(ExtrudeConstants.IN_POSITION)),
            new InstantCommand(() -> intake.requestTransition(Intake.State.EXTRUDE_IN))));
    registerStateCommand(
        State.EXTRUDE_OUT,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setExtruderPosition(ExtrudeConstants.OUT_POSITION)),
            new InstantCommand(() -> intake.requestTransition(Intake.State.FEED_IN))));
    registerStateCommand(State.MANUAL_EXTRUDE, new MoveExtrude(this));
    registerStateCommand(
        State.AGITATE,
        new InstantCommand(
            () -> {
              agitateOut = true;
              io.setExtruderVelocity(-ExtrudeConstants.AGITATE_VELOCITY);
              intake.requestTransition(Intake.State.FEED_IN);
            }));
  }

  public void addExtruderPosition(double delta) {
    io.addExtruderPosition(delta);
  }

  public void registerStateTransition() {
    addOmniTransition(State.UNDETERMINED);
    addOmniTransition(State.IDLE);
    addOmniTransition(State.EXTRUDE_OUT);
    addOmniTransition(State.EXTRUDE_IN);
    addOmniTransition(State.MANUAL_EXTRUDE);
    addOmniTransition(State.AGITATE);
  }

  @Override
  public void determineSelf() {
    setState(State.IDLE);
  }

  public double getPos() {
    return inputs.encoderPosition;
  }

  public double getDesiredPos() {
    if (getState().equals(State.EXTRUDE_IN)) {
      desiredPos = ExtrudeConstants.IN_POSITION;
    } else if (getState().equals(State.EXTRUDE_OUT)) {
      desiredPos = ExtrudeConstants.OUT_POSITION;
    } else if (getState().equals(State.MANUAL_EXTRUDE)) {
      desiredPos = io.getTargetPosition();
    }
    return desiredPos;
  }

  @Override
  protected void update() {
    // Update the state of the subsystem
    inputs.state = this.getState();
    io.updateInputs(inputs);

    if (getState() == State.AGITATE) {
      updateAgitateDynamic();
    } else {
      if (io.getPosition() > -3) {
        intake.requestTransition(Intake.State.IDLE);
        autoIntake = false;
      } else if (io.getPosition() < -23 && !autoIntake) {
        autoIntake = true;
        intake.requestTransition(Intake.State.FEED_IN);
      }
    }
  }

  /**
   * Dynamic agitate: pushes inward until motor current nears the limit (hit resistance), then
   * bounces back out to the out position. Slows down when approaching the in limit.
   */
  private void updateAgitateDynamic() {
    double pos = io.getPosition();
    double current = Math.abs(inputs.current);

    if (agitateOut) {
      // Moving outward — check if we've reached the out position
      if (pos <= ExtrudeConstants.AGITATE_OUT_POS) {
        agitateOut = false;
        io.setExtruderVelocity(ExtrudeConstants.AGITATE_VELOCITY);
      }
    } else {
      // Moving inward — check if current is spiking (hit resistance) or reached in pos
      if (current >= ExtrudeConstants.AGITATE_CURRENT_THRESHOLD
          || pos >= ExtrudeConstants.AGITATE_IN_POS) {
        agitateOut = true;
        io.setExtruderVelocity(-ExtrudeConstants.AGITATE_VELOCITY);
      }
      // Approaching the in limit — ease off the speed
      else if (pos >= ExtrudeConstants.AGITATE_IN_POS - ExtrudeConstants.AGITATE_SLOW_DISTANCE) {
        io.setExtruderVelocity(ExtrudeConstants.AGITATE_SLOW_VELOCITY);
      }
    }
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    EXTRUDE_IN,
    EXTRUDE_OUT,
    MANUAL_EXTRUDE,
    AGITATE;
  }
}
