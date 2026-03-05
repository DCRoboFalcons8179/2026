package frc.robot.subsystems.shooter;

import static frc.robot.Constants.C_Shooter.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {

  private Timer timer = new Timer();

  protected final TalonFXS leadShooter = new TalonFXS(LEAD_SHOOTER_ID);

  public ShooterIOReal() {
    configureMotor();
    timer.reset();
  }

  private void configureMotor() {
    // Lead configuration
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
    // Pitch config
    Slot0Configs leadShooterConfig =
        new Slot0Configs().withKP(LEAD_KP).withKI(LEAD_KI).withKD(LEAD_KD);

    leadShooter.getConfigurator().apply(leadShooterConfig);
  }

  @Override
  public void stop() {
    leadShooter.set(0);
    leadShooter.stopMotor();
  }

  @Override
  public void updateInputs(ShooterInputsAutoLogged inputs) {
    inputs.current = leadShooter.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = leadShooter.getMotorVoltage().getValueAsDouble();
    inputs.velocity = leadShooter.getVelocity().getValueAsDouble();
    inputs.encoderPosition = leadShooter.getPosition().getValueAsDouble();

    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public boolean isCharged() {
    double omega = leadShooter.getVelocity().getValueAsDouble();

    return omega >= OUTPUT_SPEED - ERROR_MARGIN;
  }
}
