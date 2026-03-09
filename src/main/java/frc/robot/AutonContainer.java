package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

public class AutonContainer {
    public static void CreateAutonChooser(LoggedDashboardChooser<Command> chooser){
        
    // Set up auton routines for the selector. make sure that the auton names match the option names to avoid confusion and typos
    //an improperly set up routine will not work and we will be very sad
    //auton names are uniform and follow what they will do to avoid potential confusion

    chooser.addOption("///CLEAR AREA///", null);
    //these 4 autons will be used in the event a team requests we have our robot do nothing for auton, just in case
    chooser.addOption(
        "A: Straight forward", new PathPlannerAuto("A-straight-out"));
    chooser.addOption(
        "B: Straight backward", new PathPlannerAuto("B-straight-back"));
    chooser.addOption(
        "C: Hub forward", new PathPlannerAuto("C-hub-out"));
    chooser.addOption(
        "D: Hub backward", new PathPlannerAuto("D-hub-back"));

    
    //this auton goes into the middle and messes up balls
    chooser.addOption(
        "00: Stupid", new PathPlannerAuto("Stupid"));

    
    chooser.addOption("///NO HANGING///", null);
    //these are the autons that do not have any hanging at the end of the process
    //these two stay in our end
    chooser.addOption(
        "01: Depot Score Human Score", new PathPlannerAuto("1-depot-score-human-score"));
    chooser.addOption(
        "02: Human Score Depot Score", new PathPlannerAuto("2-human-score-depot-score"));
    //these four stay on one side and travel to neutral for collection
    chooser.addOption(
        "03: Depot Hub Score Neutral Hub Score", new PathPlannerAuto("3-depot-hub-score-neutral-hub-score"));
    chooser.addOption(
        "04: Human Hub Score Neutral Hub Score", new PathPlannerAuto("4-human-hub-score-neutral-hub-score"));
    chooser.addOption(
        "05: Hub Depot Score Neutral Score", new PathPlannerAuto("5-hub-depot-score-neutral-score"));
    chooser.addOption(
        "06: Hub Human Score Neutral Score", new PathPlannerAuto("6-hub-human-score-neutral-score"));
    //these two will have the robot go in a big loop across neutral to maximize collection
    chooser.addOption(
        "07: Hub Neutral (Depot) Collect All Score", new PathPlannerAuto("7-hub-neutral-high-sweep-score"));
    chooser.addOption(
        "08: Hub Neutral (Human) Collect All Score", new PathPlannerAuto("8-hub-neutral-low-sweep-score"));

    chooser.addOption("///HANGING///", null);
    //these autons have hagning at the end
    //these two cycle between the hub and refill station
    chooser.addOption(
        "09: Hub Score Depot Score Hang", new PathPlannerAuto("9-hub-score-depot-score-hang"));
    chooser.addOption(
        "10: Hub Score Human Score Hang", new PathPlannerAuto("10-hub-score-human-score-hang"));
    //these go straight to neutral before scoring
    chooser.addOption(
        "11: Lower Neutral Score Hang", new PathPlannerAuto("11-lower-neutral-score-hang"));
    chooser.addOption(
        "12: Upper Neutral Score Hang", new PathPlannerAuto("12-upper-neutral-score-hang"));
    //these two go to filling station and then score, refill in neutral, and then score again before hanging
    chooser.addOption(
        "13: Lower Human Score Collect Score Hang", new PathPlannerAuto("13-lower-human-score-collect-score-hang"));
    chooser.addOption(
        "14: Upper Depot Score Collect Score Hang", new PathPlannerAuto("14-upper-depot-score-collect-score-hang"));
    //these two start at the hub, score, go to neutral, score again, then hang
    chooser.addOption(
        "15: Lower Hub Score Collect Score Hang", new PathPlannerAuto("15-lower-hub-score-collect-score-hang"));
    chooser.addOption(
        "16: Upper Hub Score Collect Score Hang", new PathPlannerAuto("16-upper-hub-score-collect-score-hang"));

    }

}
