package frc.robot.subsystems.Hanger;

import static frc.robot.Constants.Hanger.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HangerIOSim implements HangerIO {

  private Timer timer = new Timer();

  private PIDController pid = new PIDController(0.1, 0.0, 0.0);

  private final DCMotorSim hangerMotorSim;

  private static final double HANGER_GEARING = 100.0; // Adjust based on your gearing ratio
  private static final double HANGER_MOI = 0.001; // kg*m^2, adjust for hanger mass

  // Applied values
  private double appliedVoltage = 0.0;
  private double targetPosition = 0.0;
  private boolean usePositionControl = false;

  public HangerIOSim() {
    hangerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(Hanger_Motor_ID), // Or getFalcon500(1), getNEO(1), etc.
                HANGER_MOI,
                HANGER_GEARING),
            DCMotor.getKrakenX60(Hanger_Motor_ID) // Must match the motor above
            );
    timer.reset();
  }

  @Override
  public void setHangerTargetVelocity(double velocity) {
    hangerMotorSim.setAngularVelocity(velocity);
  }

  @Override
  public void setPIDControl(double setpoint) {
    pid.setSetpoint(setpoint);
    hangerMotorSim.setInputVoltage(pid.calculate(hangerMotorSim.getAngularVelocityRPM()));
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    usePositionControl = false;
    pid.reset();
  }

  @Override
  public void updateInputs(HangerInputs inputs) {

    // Update simulation with applied voltage
    hangerMotorSim.setInputVoltage(appliedVoltage);

    // Update simulation (assuming 20ms loop time)
    hangerMotorSim.update(0.02);
    // Update inputs
    inputs.current = hangerMotorSim.getCurrentDrawAmps();
    inputs.encoderPosition = hangerMotorSim.getAngularPositionRotations();
    inputs.velocity = hangerMotorSim.getAngularVelocityRPM() / 60.0; // rotations per second
  }

  @Override
  public boolean isMaxHeight() {
    // return whether or not the hanger is at its maximum height, which is when the simulated
    // encoder position is at the maximum height
    double omega = hangerMotorSim.getAngularPositionRotations();

    return omega >= maximum_height - ERROR_MARGIN;
  }

  @Override
  public boolean isMinHeight() {
    // return whether or not the hanger is at its minimum height, which is when the simulated
    // encoder position is at 0
    double omega = hangerMotorSim.getAngularPositionRotations();

    return omega <= minimum_height + ERROR_MARGIN;
  }
}
