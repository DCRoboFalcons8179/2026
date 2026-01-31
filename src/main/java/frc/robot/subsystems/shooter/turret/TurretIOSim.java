package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.C_Shooter.C_Turret;

public class TurretIOSim implements TurretIO {
  // Simulation object
  private final DCMotorSim turretSim;

  // PID controller for motion magic simulation
  private final PIDController positionController;

  // Applied values
  private double appliedVoltage = 0.0;
  private double targetPosition = 0.0;
  private boolean usePositionControl = false;

  // Simulation constants - ADJUST THESE TO MATCH YOUR ROBOT
  private static final double TURRET_GEARING = 100.0; // Adjust based on your gearing ratio
  private static final double TURRET_MOI = 0.001; // kg*m^2, adjust for your turret mass

  public TurretIOSim() {
    // Initialize turret sim using LinearSystemId
    // The DCMotorSim constructor requires a LinearSystem, DCMotor, and optional measurement noise
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), // Or getFalcon500(1), getNEO(1), etc.
                TURRET_MOI,
                TURRET_GEARING),
            DCMotor.getKrakenX60(1) // Must match the motor above
            );

    // Initialize PID controller with your real constants
    positionController =
        new PIDController(C_Turret.TURRET_KP, C_Turret.TURRET_KI, C_Turret.TURRET_KD);

    setPIDControl();
  }

  @Override
  public void setPIDControl() {
    // Update PID gains if needed
    positionController.setPID(C_Turret.TURRET_KP, C_Turret.TURRET_KI, C_Turret.TURRET_KD);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    usePositionControl = false;
    positionController.reset();
  }

  @Override
  public void updateInputs(TurretInputs inputs) {
    // If using position control, calculate voltage from PID
    if (usePositionControl) {
      double currentPosition = turretSim.getAngularPositionRotations();
      double pidOutput = positionController.calculate(currentPosition, targetPosition);

      // Clamp to reasonable voltage limits
      appliedVoltage = Math.max(-12.0, Math.min(12.0, pidOutput));
    }

    // Update simulation with applied voltage
    turretSim.setInputVoltage(appliedVoltage);

    // Update simulation (assuming 20ms loop time)
    turretSim.update(0.02);

    // Update inputs
    inputs.current = turretSim.getCurrentDrawAmps();
    inputs.encoderPosition = turretSim.getAngularPositionRotations();
    inputs.velocity = turretSim.getAngularVelocityRPM() / 60.0; // rotations per second
    inputs.appliedVoltage = appliedVoltage;
    inputs.targetPosition = targetPosition;
    inputs.atTarget =
        usePositionControl
            && Math.abs(inputs.encoderPosition - targetPosition) < 0.01; // Within 0.01 rotations
  }

  @Override
  public void moveTurret(double position) {
    // Simulate MotionMagicVoltage control
    targetPosition = position;
    usePositionControl = true;
  }

  @Override
  public void moveTurretPO(double omegaPercent) {
    // Simulate DutyCycleOut control (percent output)
    // Convert percent (-1 to 1) to voltage (-12 to 12)
    appliedVoltage = omegaPercent * 12.0;
    usePositionControl = false;
  }
}
