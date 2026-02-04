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
    hangerMotor.set(velocity);
  }

  @Override
  public void setPIDControl(double setpoint) {
    pid.setSetpoint(setpoint);
    hangerMotor.setVoltage(pid.calculate(hangerMotor.getTorqueCurrent().getValueAsDouble()));
  }

  @Override
  public void stop() {
    hangerMotor.set(0);
    hangerMotor.stopMotor();
  }

  @Override
  public void updateInputs(HangerInputs inputs) {
    inputs.current = hangerMotor.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = hangerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public boolean isMaxHeight() {
    double omega = hangerMotor.getPosition().getValueAsDouble();

    return omega >= maximum_height;
  }
}
