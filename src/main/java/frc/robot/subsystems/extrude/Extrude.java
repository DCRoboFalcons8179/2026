// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extrude;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.SMF.StateMachine;
import frc.robot.commands.MoveExtrude;
import frc.robot.subsystems.extrude.ExtrudeIO.ExtrudeInputs;

public class Extrude extends StateMachine<Extrude.State> {
  /** Creates a new Intake. */
  public final ExtrudeIO io;

  public final ExtrudeInputs inputs = new ExtrudeInputs();

  protected final TalonFXS extruder = new TalonFXS(Constants.Intake.EXTRUDER_ID);

  public Extrude(ExtrudeIO io) {
    super("Extruder", State.UNDETERMINED, State.class);
    this.io = io;

    io.updateInputs(inputs);

    registerStateCommand();
    registerStateTransition();
  }

  public void registerStateCommand() {
    registerStateCommand(State.IDLE, new InstantCommand(io::stop));
    registerStateCommand(
        State.EXTRUDE_IN,
        new InstantCommand(() -> io.setExtruderPosition(Constants.Intake.EXTRUDER_IN_POSITION)));
    registerStateCommand(
        State.EXTRUDE_OUT,
        new SequentialCommandGroup(
            new InstantCommand(
                () -> io.setExtruderPosition(Constants.Intake.EXTRUDER_OUT_POSITION)),
            new MoveExtrude(null)));
  }

  public void registerStateTransition() {
    addOmniTransition(State.UNDETERMINED);
    addOmniTransition(State.IDLE);
    addOmniTransition(State.EXTRUDE_OUT);
  }

  @Override
  public void determineSelf() {
    setState(State.IDLE);
  }

  public double getPos() {
    return extruder.getPosition().getValueAsDouble();
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    SmartDashboard.putString("Extrude State", getState().toString());
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    EXTRUDE_IN,
    EXTRUDE_OUT,
  }
}
