package frc.robot.subsystems.Hanger;

import static frc.robot.Constants.Hanger.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class HangerIOReal implements HangerIO {

  private Timer timer = new Timer();

  private PIDController pid = new PIDController(0.1, 0.0, 0.0);

  protected final TalonFXS hangerMotor = new TalonFXS(Hanger_Motor_ID);

  public HangerIOReal() {
    configureMotor();
    timer.reset();
  }

  private void configureMotor() {
    hangerMotor.getConfigurator().apply(CURRENT_LIMIT);
    hangerMotor.setNeutralMode(NEUTRAL_MODE);
  }

  @Override
  public void setHangerTargetVelocity(double velocity) {
    //give voltage to the motor based on the commanded speed
    hangerMotor.set(velocity);
  }

  @Override
  public void setPIDControl(double setpoint) {
    //set the setpoint for the PID controller, and calculate the output based on the current position of the motor, and set the voltage to that output
    pid.setSetpoint(setpoint);
    hangerMotor.setVoltage(pid.calculate(hangerMotor.getTorqueCurrent().getValueAsDouble()));
  }

  @Override
  public void stop() {
    //stop the motor by setting the output to 0
    hangerMotor.set(0);
    hangerMotor.stopMotor();
  }

  @Override
  public void updateInputs(HangerInputs inputs) {
    //update the inputs by reading from the motor's sensors
    inputs.current = hangerMotor.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = hangerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public boolean isMaxHeight() {
    //return whether or not the hanger is at its maximum height, which is when the encoder position is at the maximum height, with some error margin to account for noise and overshooting
    double omega = hangerMotor.getPosition().getValueAsDouble();

    return omega >= maximum_height - ERROR_MARGIN;
  }

  @Override
  public boolean isMinHeight() {
    //return whether or not the hanger is at its minimum height, which is when the encoder position is at 0, with some error margin to account for noise and overshooting
    double omega = hangerMotor.getPosition().getValueAsDouble();

    return omega <= minimum_height + ERROR_MARGIN;
  }
}
