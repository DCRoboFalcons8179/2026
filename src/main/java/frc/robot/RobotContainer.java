// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up auton routines for the selector. make sure that the auton names match the option names to avoid confusion and typos
    //an improperly set up routine will not work and we will be very sad
    //auton names are uniform and follow what they will do to avoid potential confusion

    autoChooser.addOption("///CLEAR AREA///", getAutonomousCommand());
    //these 4 autons will be used in the event a team requests we have our robot do nothing for auton, just in case
    autoChooser.addOption(
        "A: Straight forward", new PathPlannerAuto("A-straight-out"));
    autoChooser.addOption(
        "B: Straight backward", new PathPlannerAuto("B-straight-back"));
    autoChooser.addOption(
        "C: Hub forward", new PathPlannerAuto("C-hub-out"));
    autoChooser.addOption(
        "D: Hub backward", new PathPlannerAuto("D-hub-back"));

    
    //this auton goes into the middle and messes up balls
    autoChooser.addOption(
        "00: Stupid", new PathPlannerAuto("Stupid"));

    
    autoChooser.addOption("///NO HANGING///", getAutonomousCommand());
    //these are the autons that do not have any hanging at the end of the process
    //these two stay in our end
    autoChooser.addOption(
        "01: Depot Score Human Score", new PathPlannerAuto("1-depot-score-human-score"));
    autoChooser.addOption(
        "02: Human Score Depot Score", new PathPlannerAuto("2-human-score-depot-score"));
    //these four stay on one side and travel to neutral for collection
    autoChooser.addOption(
        "03: Depot Hub Score Neutral Hub Score", new PathPlannerAuto("3-depot-hub-score-neutral-hub-score"));
    autoChooser.addOption(
        "04: Human Hub Score Neutral Hub Score", new PathPlannerAuto("4-human-hub-score-neutral-hub-score"));
    autoChooser.addOption(
        "05: Hub Depot Score Neutral Score", new PathPlannerAuto("5-hub-depot-score-neutral-score"));
    autoChooser.addOption(
        "06: Hub Human Score Neutral Score", new PathPlannerAuto("6-hub-human-score-neutral-score"));
    //these two will have the robot go in a big loop across neutral to maximize collection
    autoChooser.addOption(
        "07: Hub Neutral (Depot) Collect All Score", new PathPlannerAuto("7-hub-neutral-high-sweep-score"));
    autoChooser.addOption(
        "08: Hub Neutral (Human) Collect All Score", new PathPlannerAuto("8-hub-neutral-low-sweep-score"));

    autoChooser.addOption("///HANGING///", getAutonomousCommand());
    //these autons have hagning at the end
    //these two cycle between the hub and refill station
    autoChooser.addOption(
        "09: Hub Score Depot Score Hang", new PathPlannerAuto("9-hub-score-depot-score-hang"));
    autoChooser.addOption(
        "10: Hub Score Human Score Hang", new PathPlannerAuto("10-hub-score-human-score-hang"));
    //these go straight to neutral before scoring
    autoChooser.addOption(
        "11: Lower Neutral Score Hang", new PathPlannerAuto("11-lower-neutral-score-hang"));
    autoChooser.addOption(
        "12: Upper Neutral Score Hang", new PathPlannerAuto("12-upper-neutral-score-hang"));
    //these two go to filling station and then score, refill in neutral, and then score again before hanging
    autoChooser.addOption(
        "13: Lower Human Score Collect Score Hang", new PathPlannerAuto("13-lower-human-score-collect-score-hang"));
    autoChooser.addOption(
        "14: Upper Depot Score Collect Score Hang", new PathPlannerAuto("14-upper-depot-score-collect-score-hang"));
    //these two start at the hub, score, go to neutral, score again, then hang
    autoChooser.addOption(
        "15: Lower Hub Score Collect Score Hang", new PathPlannerAuto("15-lower-hub-score-collect-score-hang"));
    autoChooser.addOption(
        "16: Upper Hub Score Collect Score Hang", new PathPlannerAuto("16-upper-hub-score-collect-score-hang"));
    

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
