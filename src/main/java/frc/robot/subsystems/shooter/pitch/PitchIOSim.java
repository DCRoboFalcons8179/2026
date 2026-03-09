package frc.robot.subsystems.shooter.pitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class PitchIOSim implements PitchIO {
  // Simulation object
  private final DCMotorSim pitchSim;

  // PID controller for motion magic simulation
  private final PIDController positionController;

  // Applied values
  private double appliedVoltage = 0.0;
  private double targetPosition = 0.0;
  private boolean usePositionControl = false;

  // Simulation constants - ADJUST THESE TO MATCH YOUR ROBOT
  private static final double PITCH_GEARING = 100.0; // Adjust based on your gearing ratio
  private static final double PITCH_MOI = 0.001; // kg*m^2, adjust for your pitch mass

  public PitchIOSim() {
    // Initialize pitch sim using LinearSystemId
    // The DCMotorSim constructor requires a LinearSystem, DCMotor, and optional measurement noise
    pitchSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1), // Or getFalcon500(1), getNEO(1), etc.
                PITCH_MOI,
                PITCH_GEARING),
            DCMotor.getKrakenX44(1) // Must match the motor above
            );

    // Initialize PID controller with your real constants
    positionController =
        new PIDController(
            PitchConstants.PITCH_KP, PitchConstants.PITCH_KI, PitchConstants.PITCH_KD);

    setPIDControl();
  }

  @Override
  public void setPIDControl() {
    // Update PID gains if needed
    positionController.setPID(
        PitchConstants.PITCH_KP, PitchConstants.PITCH_KI, PitchConstants.PITCH_KD);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    usePositionControl = false;
    positionController.reset();
  }

  @Override
  public void updateInputs(PitchInputsAutoLogged inputs) {
    // If using position control, calculate voltage from PID
    if (usePositionControl) {
      double currentPosition = pitchSim.getAngularPositionRotations();
      double pidOutput = positionController.calculate(currentPosition, targetPosition);

      // Clamp to reasonable voltage limits
      appliedVoltage = Math.max(-12.0, Math.min(12.0, pidOutput));
    }

    // Update simulation with applied voltage
    pitchSim.setInputVoltage(appliedVoltage);

    // Update simulation (assuming 20ms loop time)
    pitchSim.update(0.02);

    // Update inputs
    inputs.current = pitchSim.getCurrentDrawAmps();
    inputs.encoderPosition = pitchSim.getAngularPositionRotations();
    inputs.velocity = pitchSim.getAngularVelocityRPM() / 60.0; // rotations per second
    inputs.appliedVoltage = appliedVoltage;
    inputs.targetPosition = targetPosition;
    inputs.atTarget =
        usePositionControl
            && Math.abs(inputs.encoderPosition - targetPosition) < 0.01; // Within 0.01 rotations

    Logger.processInputs("Pitch", inputs);
  }

  @Override
  public void movePitch(double position) {
    // Simulate MotionMagicVoltage control
    targetPosition = position;
    usePositionControl = true;
  }

  @Override
  public double getPitchPosition() {
    return pitchSim.getAngularPositionRotations();
  }
}
