package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.math.Range;
import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {
  protected final TalonFXS shooter = new TalonFXS(ShooterConstants.ID);
  private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0).withSlot(0);
  protected final TalonFXS feeder = new TalonFXS(ShooterConstants.FEED_ID);

  private double targetVelocity = 0;

  public ShooterIOReal() {
    configureMotor();
  }

  private void configureMotor() {
    // Lead configuration
    shooter.getConfigurator().apply(ShooterConstants.CURRENT_LIMIT);
    shooter.setNeutralMode(ShooterConstants.NEUTRAL_MODE);
    shooter.getConfigurator().apply(new MotorOutputConfigs().withInverted(ShooterConstants.INVERT));

    feeder.getConfigurator().apply(ShooterConstants.CURRENT_LIMIT);
    feeder.setNeutralMode(ShooterConstants.NEUTRAL_MODE);
    feeder
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(ShooterConstants.FEED_INVERT));

    setPIDControl();
  }

  @Override
  public void setShooterTargetVelocity(double velocity) {
    targetVelocity = velocity;

    shooter.setControl(
        shooterVelocityRequest.withVelocity(velocity).withAcceleration(5.0).withEnableFOC(false));

    // If the shooter is charged, run the feeder
    if (isCharged()) {
      feeder.set(ShooterConstants.FEED_OUTPUT_SPEED);
    }
  }

  @Override
  public void setPIDControl() {
    // Pitch config
    Slot0Configs leadShooterConfig =
        new Slot0Configs()
            .withKP(ShooterConstants.KP)
            .withKI(ShooterConstants.KI)
            .withKD(ShooterConstants.KD)
            .withKV(ShooterConstants.KV);

    shooter.getConfigurator().apply(leadShooterConfig);

    Slot0Configs shootFeedConfig =
        new Slot0Configs()
            .withKP(ShooterConstants.FEED_KP)
            .withKI(ShooterConstants.FEED_KI)
            .withKD(ShooterConstants.FEED_KD);

    feeder.getConfigurator().apply(shootFeedConfig);
  }

  @Override
  public void stop() {
    // shooter.set(0);
    shooter.setControl(
        shooterVelocityRequest.withVelocity(0).withAcceleration(1.0).withEnableFOC(false));
    shooter.stopMotor();

    feeder.set(0);
    feeder.stopMotor();
  }

  @Override
  public void updateInputs(ShooterInputsAutoLogged inputs) {
    inputs.current = shooter.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = shooter.getMotorVoltage().getValueAsDouble();
    inputs.velocity = shooter.getVelocity().getValueAsDouble();
    inputs.encoderPosition = shooter.getPosition().getValueAsDouble();
    inputs.targetVelocity = targetVelocity;

    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public boolean isCharged() {
    double omega = shooter.getVelocity().getValueAsDouble();

    return Range.inRange(
        omega * ShooterConstants.GEAR_RATIO, ShooterConstants.ERROR_MARGIN, targetVelocity);
  }

  @Override
  public double getVelocity() {
    return shooter.getVelocity().getValueAsDouble();
  }
}
