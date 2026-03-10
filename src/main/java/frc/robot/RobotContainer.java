// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.extrude.Extrude;
import frc.robot.subsystems.extrude.ExtrudeIO;
import frc.robot.subsystems.extrude.ExtrudeIOReal;
import frc.robot.subsystems.extrude.ExtrudeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.pitch.Pitch;
import frc.robot.subsystems.shooter.pitch.PitchIO;
import frc.robot.subsystems.shooter.pitch.PitchIOReal;
import frc.robot.subsystems.shooter.pitch.PitchIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOReal;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private final Vision vision;
  private final Turret turret;
  private final Shooter shooter;
  private final Pitch pitch;
  private final Intake intake;
  private final Extrude extrude;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick box = new CommandJoystick(1);

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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0)
                // new VisionIOPhotonVision(
                // VisionConstants.camera1Name, VisionConstants.robotToCamera1));
                );

        turret = new Turret(new TurretIOReal(), vision);

        shooter = new Shooter(new ShooterIOReal());

        pitch = new Pitch(new PitchIOReal());
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

        intake = new Intake(new IntakeIOReal());
        extrude = new Extrude(new ExtrudeIOReal());

        // Enable state machines
        intake.enable();
        extrude.enable();
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
        shooter = new Shooter(new ShooterIO() {});
        pitch = new Pitch(new PitchIOSim());
        intake = new Intake(new IntakeIOSim() {});
        extrude = new Extrude(new ExtrudeIOSim() {});

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

        shooter = new Shooter(new ShooterIO() {});

        pitch = new Pitch(new PitchIO() {});

        intake = new Intake(new IntakeIO() {});

        extrude = new Extrude(new ExtrudeIO() {});
        break;
    }

    enableStateSubsystems();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    AutonContainer.CreateAutonChooser(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
    configureNamedCommands();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
        "aimToTag",
        DriveCommands.cameraDrive(
            drive, vision, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    NamedCommands.registerCommand(
        "Shooter Charge",
        new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE)));
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
        new InstantCommand(() -> turret.requestTransition(Turret.State.UNLOCKED)));

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

    pitch.enable();
    pitch.determineState();

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

    // Lock on to tag
    controller
        .a()
        .whileTrue(
            DriveCommands.cameraDrive(
                drive, vision, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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
        .povDown()
        .onTrue(new InstantCommand(() -> turret.requestTransition(Turret.State.AIM)))
        .onFalse(new InstantCommand(() -> turret.requestTransition(Turret.State.UNLOCKED)));

    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  turret.setTurretPose(-3);
                }))
        .onFalse(new InstantCommand(() -> turret.setTurretPose(0)));

    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  turret.setTurretPose(3);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  turret.setTurretPose(0);
                }));

    controller
        .rightBumper()
        .onTrue(new InstantCommand(() -> pitch.setPitchPose(100)))
        .onFalse(new InstantCommand(() -> pitch.setPitchPose(0)));

    // Feeds intake in when b button is pressed
    controller
        .b()
        .toggleOnTrue(new InstantCommand(() -> intake.requestTransition(Intake.State.FEED_IN)));
    // Feeds intake out when right bumper is pressed
    controller
        .b()
        .toggleOnTrue(new InstantCommand(() -> intake.requestTransition(Intake.State.IDLE)));

    controller
        .leftTrigger()
        .onTrue(new InstantCommand(() -> extrude.requestTransition(Extrude.State.EXTRUDE_OUT)));
    controller
        .leftBumper()
        .onTrue(new InstantCommand(() -> extrude.requestTransition(Extrude.State.EXTRUDE_IN)));
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> extrude.addExtruderPosition(Constants.Extruder.EXTRUDER_MANAL_DELTA)));

    box.button(1)
        .onTrue(new InstantCommand(() -> shooter.requestTransition(Shooter.State.CHARGE)))
        .onFalse(new InstantCommand(() -> shooter.requestTransition(Shooter.State.IDLE)));
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
