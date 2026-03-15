// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.BinaryToInt;
import frc.robot.GetAuton;
import frc.robot.subsystems.extrude.Extrude;
import java.io.IOException;
import org.json.simple.parser.ParseException;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CheckAutonOptions extends InstantCommand {
  private final CommandJoystick boxLeft;
  private final CommandJoystick boxRight;
  private final Extrude extrude;
  PathPlannerPath path = null;

  public CheckAutonOptions(CommandJoystick boxLeft, CommandJoystick boxRight, Extrude extrude) {
    this.boxLeft = boxLeft;
    this.boxRight = boxRight;
    this.extrude = extrude;
    addRequirements(extrude);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean trench = boxLeft.button(3).getAsBoolean();
    boolean ramp = boxRight.button(4).getAsBoolean();

    String humanOrDepo = getAutoSide(boxLeft, boxRight);

    if (humanOrDepo.equals("None")) return;

    try {
      path = PathPlannerPath.fromPathFile(getPathName(boxLeft, boxRight));
    } catch (FileVersionException | IOException | ParseException e) {
      System.out.println("Path Not Found!");
    }

    if (trench || ramp) {
      var auto = AutoBuilder.followPath(path);
      // Before path runs
      // .beforeStarting(
      //     new InstantCommand(() -> extrude.requestTransition(Extrude.State.EXTRUDE_IN)))
      // // After path runs
      // .andThen(
      //     new InstantCommand(() -> extrude.requestTransition(Extrude.State.EXTRUDE_OUT)));

      CommandScheduler.getInstance().schedule(auto);
    }
  }

  public static String getPathName(CommandJoystick boxLeft, CommandJoystick boxRight) {
    boolean trench = boxLeft.button(3).getAsBoolean();
    boolean ramp = boxRight.button(4).getAsBoolean();

    String humanOrDepo = getAutoSide(boxLeft, boxRight);
    String path = "None";

    if (humanOrDepo.equals("None")) {
      return path;
    }

    // If the trench and the ramp are selected or the trench is selected, run the
    // trench auton
    if (trench && ramp || trench) {
      path = humanOrDepo + " Trench Out";
    } else if (ramp) {
      path = humanOrDepo + " Ramp Out";
    }

    return path;
  }

  public static String getAutoSide(CommandJoystick boxLeft, CommandJoystick boxRight) {
    String autoName = GetAuton.getAutonName(BinaryToInt.getInt(boxRight, boxLeft)).toLowerCase();

    String humanOrDepo;
    if (autoName.contains("c(hub)") || autoName.contains("do nothing") || autoName.equals("crew")) {
      return "None";
    } else {
      humanOrDepo = autoName.contains("human") ? "Human" : "Depo";
    }

    return humanOrDepo;
  }
}
