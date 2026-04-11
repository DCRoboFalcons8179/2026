package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.subsystems.Music;
import frc.robot.subsystems.bellyBeaterBar.BellyBeaterBar;
import frc.robot.subsystems.bellyBeaterBar.BellyBeaterBar.State;
import frc.robot.subsystems.shooter.pitch.PitchConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {
  private final BellyBeaterBar bellyBeaterBar;

  protected final TalonFXS shooter = new TalonFXS(ShooterConstants.ID);
  protected final TalonFXS follower = new TalonFXS(PitchConstants.PITCH_ID);
  private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage followerVelocityRequest = new VelocityVoltage(0).withSlot(0);
  protected final TalonFXS feeder = new TalonFXS(ShooterConstants.FEED_ID);

  private double mainTargetVelocity = 0;
  private double followerTargetVelocity = 0;
  

  public ShooterIOReal(BellyBeaterBar bellyBeaterBar) {
    this.bellyBeaterBar = bellyBeaterBar;

    configureMotor();

    Music.addMotor(shooter);
    Music.addMotor(follower);
    Music.addMotor(feeder);
  }

  private void configureMotor() {
    // Lead configuration
    shooter.getConfigurator().apply(ShooterConstants.CURRENT_LIMIT);
    shooter.setNeutralMode(ShooterConstants.NEUTRAL_MODE);
    shooter.getConfigurator().apply(new MotorOutputConfigs().withInverted(ShooterConstants.INVERT));

    feeder.getConfigurator().apply(ShooterConstants.CURRENT_LIMIT);
    feeder.setNeutralMode(ShooterConstants.FOLLOWER_NEUTRAL_MODE);
    feeder
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(ShooterConstants.FEED_INVERT));

    setPIDControl();
  }

  @Override
  public void setShooterTargetVelocity(double velocity) {
    // Main velocity
    mainTargetVelocity = 30;

    // Secondary Velocity
    followerTargetVelocity = velocity;

    shooter.setControl(
        shooterVelocityRequest
            .withVelocity(mainTargetVelocity)
            .withAcceleration(5.0)
            .withEnableFOC(false));

    follower.setControl(
        followerVelocityRequest
            .withVelocity(followerTargetVelocity)
            .withAcceleration(5.0)
            .withEnableFOC(false));

    // If the shooter is charged, run the feeder
    if (isCharged()) {
      feeder.set(ShooterConstants.FEED_OUTPUT_SPEED);
      bellyBeaterBar.requestTransition(State.OUT);
    } else {
      bellyBeaterBar.requestTransition(State.IDLE);
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

    Slot0Configs followShooterConfig =
        new Slot0Configs()
            .withKP(ShooterConstants.FOLLOWER_KP)
            .withKI(ShooterConstants.FOLLOWER_KI)
            .withKD(ShooterConstants.FOLLOWER_KD)
            .withKV(ShooterConstants.FOLLOWER_KV);

    follower.getConfigurator().apply(followShooterConfig);

    Slot0Configs shootFeedConfig =
        new Slot0Configs()
            .withKP(ShooterConstants.FEED_KP)
            .withKI(ShooterConstants.FEED_KI)
            .withKD(ShooterConstants.FEED_KD)
            .withKV(ShooterConstants.FEED_KV);

    feeder.getConfigurator().apply(shootFeedConfig);
  }

  @Override
  public void stop() {
    // shooter.set(0);
    shooter.setControl(
        shooterVelocityRequest.withVelocity(0).withAcceleration(1.0).withEnableFOC(false));
    shooter.stopMotor();

    follower.set(0);
    follower.stopMotor();

    feeder.set(0);
    feeder.stopMotor();

    bellyBeaterBar.requestTransition(State.IDLE);
  }

  @Override
  public void beaterBarReverse(double velocity) {
    feeder.set(velocity);
  }

  @Override
  public void updateInputs(ShooterInputsAutoLogged inputs) {
    inputs.current = shooter.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = shooter.getMotorVoltage().getValueAsDouble();
    inputs.mainVelocity = shooter.getVelocity().getValueAsDouble();
    inputs.mainTargetVelocity = mainTargetVelocity;
    inputs.followerVelocity = follower.getVelocity().getValueAsDouble();
    inputs.followerTargetVelocity = followerTargetVelocity;
    inputs.atSpeed = isCharged();

    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public boolean isCharged() {
    double omega = follower.getVelocity().getValueAsDouble();

    boolean atSpeed = (omega * ShooterConstants.GEAR_RATIO) + 10 > followerTargetVelocity;

    return atSpeed;
  }

  @Override
  public double getVelocity() {
    return shooter.getVelocity().getValueAsDouble();
  }
}
