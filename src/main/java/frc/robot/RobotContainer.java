// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CheckAutonOptions;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.extrude.Extrude;
import frc.robot.subsystems.extrude.ExtrudeConstants;
import frc.robot.subsystems.extrude.ExtrudeIO;
import frc.robot.subsystems.extrude.ExtrudeIOReal;
import frc.robot.subsystems.extrude.ExtrudeIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOReal;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Turret turret;
  private final Shooter shooter;
  private final Intake intake;
  private final Extrude extrude;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick boxLeft = new CommandJoystick(1);
  private final CommandJoystick boxRight = new CommandJoystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureNamedCommands();

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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));

        turret = new Turret(new TurretIOReal(), vision);

        shooter = new Shooter(new ShooterIOReal(), vision);

        intake = new Intake(new IntakeIOReal());
        extrude = new Extrude(new ExtrudeIOReal(), intake);
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        turret = new Turret(new TurretIOSim(), vision);
        shooter = new Shooter(new ShooterIO() {}, vision);
        intake = new Intake(new IntakeIOSim() {});
        extrude = new Extrude(new ExtrudeIOSim() {}, intake);

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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        turret = new Turret(new TurretIO() {}, vision);

        shooter = new Shooter(new ShooterIO() {}, vision);

        intake = new Intake(new IntakeIO() {});

        extrude = new Extrude(new ExtrudeIO() {}, intake);
        break;
    }

    enableStateSubsystems();

    // Configure the button bindings
    configureButtonBindings();
  }

  public void periodic() {
    int autonID = BinaryToInt.getInt(boxRight, boxLeft);

    SmartDashboard.putNumber("Auton Number", autonID);
    SmartDashboard.putString("Auton Name", GetAuton.getAutonName(autonID));

    SmartDashboard.putString("Auto Side", CheckAutonOptions.getAutoSide(boxLeft, boxRight));
    SmartDashboard.putString("Path Name", CheckAutonOptions.getPathName(boxLeft, boxRight));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
        "aimToTag",
        DriveCommands.cameraDrive(
            drive, vision, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    NamedCommands.registerCommand(
        "Shooter Charge",
        new SequentialCommandGroup(
            new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE)),
            new InstantCommand(() -> intake.requestTransition(Intake.State.FEED_IN))));
    NamedCommands.registerCommand(
        "Shooter Idle", new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE)));

    NamedCommands.registerCommand(
        "Extrude Out",
        new InstantCommand(() -> extrude.requestTransition(Extrude.State.EXTRUDE_OUT)));
    NamedCommands.registerCommand(
        "Extrude In",
        new InstantCommand(() -> extrude.requestTransition(Extrude.State.EXTRUDE_IN)));

    NamedCommands.registerCommand(
        "Turret Aim Enable", new InstantCommand(() -> turret.requestTransition(Turret.State.AIM)));
    NamedCommands.registerCommand(
        "Turret Aim Disable",
        new SequentialCommandGroup(
            new InstantCommand(() -> turret.requestTransition(Turret.State.UNLOCKED)),
            new InstantCommand(() -> turret.incrementTurret(-TurretConstants.NUDGE_AMOUNT)),
            new InstantCommand(() -> turret.incrementTurret(TurretConstants.NUDGE_AMOUNT))));

    NamedCommands.registerCommand(
        "Turret Enable", new InstantCommand(() -> turret.requestTransition(Turret.State.UNLOCKED)));

    NamedCommands.registerCommand(
        "Turret Nudge Left",
        new InstantCommand(() -> turret.incrementTurret(-TurretConstants.NUDGE_AMOUNT)));

    NamedCommands.registerCommand(
        "Turret Nudge Right",
        new InstantCommand(() -> turret.incrementTurret(TurretConstants.NUDGE_AMOUNT)));

    NamedCommands.registerCommand(
        "Intake Enable", new InstantCommand(() -> intake.requestTransition(Intake.State.FEED_IN)));
    NamedCommands.registerCommand(
        "Intake Disable", new InstantCommand(() -> intake.requestTransition(Intake.State.IDLE)));
  }

  private void enableStateSubsystems() {
    turret.enable();
    turret.determineState();

    shooter.enable();
    shooter.determineState();

    intake.enable();
    intake.determineSelf();

    extrude.enable();
    extrude.determineSelf();
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

    // Switch to X pattern when X button is pressed
    controller.b().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .leftTrigger()
        .onTrue(new InstantCommand(() -> turret.requestTransition(Turret.State.AIM)))
        .onFalse(new InstantCommand(() -> turret.requestTransition(Turret.State.UNLOCKED)));

    controller
        .rightBumper()
        .toggleOnTrue(new InstantCommand(() -> intake.requestTransition(Intake.State.FEED_IN)));
    controller
        .rightBumper()
        .toggleOnTrue(new InstantCommand(() -> intake.requestTransition(Intake.State.IDLE)));

    controller
        .leftBumper()
        .toggleOnTrue(
            new InstantCommand(() -> extrude.requestTransition(Extrude.State.EXTRUDE_IN)));
    controller
        .leftBumper()
        .toggleOnTrue(
            new InstantCommand(() -> extrude.requestTransition(Extrude.State.EXTRUDE_OUT)));
    controller
        .povUp()
        .onTrue(
            new InstantCommand(() -> extrude.addExtruderPosition(ExtrudeConstants.MANUAL_DELTA)));

    controller
        .povDown()
        .onTrue(
            new InstantCommand(() -> extrude.addExtruderPosition(-ExtrudeConstants.MANUAL_DELTA)));

    // Turret Manual Bump
    controller
        .povLeft()
        .onTrue(new InstantCommand(() -> turret.incrementTurret(-TurretConstants.NUDGE_AMOUNT)));
    controller
        .povRight()
        .onTrue(new InstantCommand(() -> turret.incrementTurret(TurretConstants.NUDGE_AMOUNT)));

    // Turret Reset to Zero
    controller.a().onTrue(new InstantCommand(() -> turret.setTurretPose(0)));

    controller
        .rightTrigger()
        .onTrue(new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE)))
        .onFalse(new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE)));

    controller
        .x()
        .onTrue(new InstantCommand(() -> shooter.requestTransition(Shooter.State.REVERSE)))
        .onFalse(new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE)));

    // Left box

    // Trench Left (velocity calculated with shooter velocity equation)
    boxLeft
        .button(8)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.TRENCH_ANGLE)),
                new InstantCommand(() -> shooter.setVelocity(ShooterConstants.TRENCH_VELOCITY)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE))));

    // Corner Left (velocity calculated with shooter velocity equation)
    boxLeft
        .button(1)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.CORNER_ANGLE)),
                new InstantCommand(() -> shooter.setVelocity(ShooterConstants.CORNER_VELOCITY)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE))));

    // Tower Shoot (velocity calculated with shooter velocity equation)
    boxLeft
        .button(12)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE)),
                new InstantCommand(() -> shooter.setVelocity(ShooterConstants.TOWER_VELOCITY)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE))));

    // Lebron Left
    boxLeft
        .button(1)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.LEBRON_ANGLE)),
                new InstantCommand(() -> shooter.setVelocity(ShooterConstants.LEBRON_VELOCITY)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE))));

    // Right box

    // Trench Right (velocity calculated with shooter velocity equation)
    boxRight
        .button(12)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(-TurretConstants.TRENCH_ANGLE)),
                new InstantCommand(() -> shooter.setVelocity(ShooterConstants.TRENCH_VELOCITY)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE))));

    // Zero turret
    boxLeft
        .button(11)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.requestTransition(Turret.State.UNLOCKED)),
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE))));

    // Corner Right (velocity calculated with shooter velocity equation)
    boxRight
        .button(8)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(-TurretConstants.CORNER_ANGLE)),
                new InstantCommand(() -> shooter.setVelocity(ShooterConstants.CORNER_VELOCITY)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE))));

    // Trench Arc
    boxRight
        .button(6)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setVelocity(ShooterConstants.TRENCH_ARC)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE)));

    // Cody Jones
    boxRight
        .button(10)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setVelocity(100)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE)));

    // Lebron Right
    boxRight
        .button(1)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(-TurretConstants.LEBRON_ANGLE)),
                new InstantCommand(() -> shooter.setVelocity(ShooterConstants.LEBRON_VELOCITY)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE))))
        .onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> turret.setTurretDegrees(TurretConstants.ZERO_ANGLE)),
                new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(GetAuton.getAutonName(BinaryToInt.getInt(boxRight, boxLeft)))
        .andThen(new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE)))
        .andThen(new CheckAutonOptions(boxLeft, boxRight, extrude));
  }
}
