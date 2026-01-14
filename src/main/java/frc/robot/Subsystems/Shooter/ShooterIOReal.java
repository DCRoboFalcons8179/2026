package frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class ShooterIOReal implements ShooterIO {

  private Timer timer = new Timer();

  private PIDController pid = new PIDController(0.1, 0.0, 0.0);

  protected final TalonFXS leadShooter = new TalonFXS(LEAD_SHOOTER_ID);

  public ShooterIOReal() {
    configureMotor();
    timer.reset();
  }

  private void configureMotor() {
    leadShooter.getConfigurator().apply(CURRENT_LIMIT);
    leadShooter.setNeutralMode(NEUTRAL_MODE);
    leadShooter.getConfigurator().apply(new MotorOutputConfigs().withInverted(LEAD_SHOOTER_INVERT));
  }

  @Override
  public void setShooterTargetVelocity(double velocity) {
    leadShooter.set(velocity);
  }

  @Override
  public void setPIDControl() {
    pid.setSetpoint(2.0);
    leadShooter.setVoltage(pid.calculate(leadShooter.getTorqueCurrent().getValueAsDouble()));
  }

  @Override
  public void stop() {
    leadShooter.set(0);
    leadShooter.stopMotor();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.current = leadShooter.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = leadShooter.getPosition().getValueAsDouble();
  }

  @Override
  public boolean isCharged() {
    double omega = leadShooter.getVelocity().getValueAsDouble();

    return omega >= OUTPUT_SPEED - ERROR_MARGIN;
  }
}
