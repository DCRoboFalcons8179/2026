package frc.robot.subsystems.shooter.pitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.C_Shooter.C_Turret;

public class PitchIOSim implements PitchIO {
  // Simulation object
  private final DCMotorSim pitchSim;

  // PID controller for motion magic simulation
  private final PIDController positionController;

  // Applied values
  private double appliedVoltage = 0.0;
  private double pitch = 0.0;
  private boolean usePositionControl = false;

  // Simulation constants - ADJUST THESE TO MATCH YOUR ROBOT
  private static final double TURRET_GEARING = 100.0; // Adjust based on your gearing ratio
  private static final double TURRET_MOI = 0.001; // kg*m^2, adjust for your pitch mass

  public PitchIOSim() {
    // Initialize pitch sim using LinearSystemId
    // The DCMotorSim constructor requires a LinearSystem, DCMotor, and optional measurement noise
    pitchSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), // Or getFalcon500(1), getNEO(1), etc.
                TURRET_MOI,
                TURRET_GEARING),
            DCMotor.getKrakenX60(1) // Must match the motor above
            );

    // Initialize PID controller with your real constants
    positionController = new PIDController(C_Turret.PITCH_KP, C_Turret.PITCH_KI, C_Turret.PITCH_KD);

    setPIDControl();
  }

  @Override
  public void setPIDControl() {
    // Update PID gains if needed
    positionController.setPID(C_Turret.PITCH_KP, C_Turret.PITCH_KI, C_Turret.PITCH_KD);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    usePositionControl = false;
    positionController.reset();
  }

  @Override
  public void updateInputs(PitchInputs inputs) {
    // If using position control, calculate voltage from PID
    if (usePositionControl) {
      double currentPosition = pitchSim.getAngularPositionRotations();
      double pidOutput = positionController.calculate(currentPosition, pitch);

      // Clamp to reasonable voltage limits
      appliedVoltage = Math.max(-12.0, Math.min(12.0, pidOutput));
    }

    // Update simulation with applied voltage
    pitchSim.setInputVoltage(appliedVoltage);

    // Update simulation (assuming 20ms loop time)
    pitchSim.update(0.02);

    // Update inputs
    inputs.encoderPosition = pitchSim.getAngularPositionRotations();
    inputs.pitch = pitch;
  }

  @Override
  public void tiltShooter(double position) {
    // Simulate MotionMagicVoltage control
    pitch = position;
    usePositionControl = true;
  }
}
