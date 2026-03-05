package frc.robot.subsystems.shooter;

import static frc.robot.Constants.C_Shooter.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {
  protected final TalonFXS shooter = new TalonFXS(LEAD_SHOOTER_ID);
  private final VelocityVoltage shootVelocityRequest = new VelocityVoltage(0).withSlot(0);
  protected final TalonFXS shootFeed = new TalonFXS(SHOOT_FEED_ID);

  public ShooterIOReal() {
    configureMotor();
  }

  private void configureMotor() {
    // Lead configuration
    shooter.getConfigurator().apply(CURRENT_LIMIT);
    shooter.setNeutralMode(NEUTRAL_MODE);
    shooter.getConfigurator().apply(new MotorOutputConfigs().withInverted(LEAD_SHOOTER_INVERT));

    shootFeed.getConfigurator().apply(CURRENT_LIMIT);
    shootFeed.setNeutralMode(NEUTRAL_MODE);
    shootFeed.getConfigurator().apply(new MotorOutputConfigs().withInverted(SHOOT_FEED_INVERT));

    setPIDControl();
  }

  @Override
  public void setShooterTargetVelocity(double velocity) {
    shooter.setControl(
        shootVelocityRequest.withVelocity(velocity).withAcceleration(5.0).withEnableFOC(false));

    // shooter.set(1);

    // If the shooter is charged, run the feeder
    if (true) {
      shootFeed.set(SHOOT_FEED_OUTPUT_SPEED);
    }
  }

  @Override
  public void setPIDControl() {
    // Pitch config
    Slot0Configs leadShooterConfig =
        new Slot0Configs()
            .withKP(SHOOTER_KP)
            .withKI(SHOOTER_KI)
            .withKD(SHOOTER_KD)
            .withKV(SHOOTER_KV);

    shooter.getConfigurator().apply(leadShooterConfig);

    Slot0Configs shootFeedConfig =
        new Slot0Configs().withKP(SHOOT_FEED_KP).withKI(SHOOT_FEED_KI).withKD(SHOOT_FEED_KD);

    shootFeed.getConfigurator().apply(shootFeedConfig);
  }

  @Override
  public void stop() {
    // shooter.set(0);
    shooter.setControl(
        shootVelocityRequest.withVelocity(0).withAcceleration(1.0).withEnableFOC(false));
    shooter.stopMotor();

    shootFeed.set(0);
    shootFeed.stopMotor();
  }

  @Override
  public void updateInputs(ShooterInputsAutoLogged inputs) {
    inputs.current = shooter.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = shooter.getMotorVoltage().getValueAsDouble();
    inputs.velocity = shooter.getVelocity().getValueAsDouble();
    inputs.encoderPosition = shooter.getPosition().getValueAsDouble();

    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public boolean isCharged() {
    double omega = shooter.getVelocity().getValueAsDouble();

    return omega >= OUTPUT_SPEED - ERROR_MARGIN;
  }

  @Override
  public double getVelocity() {
    return shooter.getVelocity().getValueAsDouble();
  }
}
