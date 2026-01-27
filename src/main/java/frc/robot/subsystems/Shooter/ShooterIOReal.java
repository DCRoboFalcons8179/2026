package frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Shooter.Turret;

public class ShooterIOReal implements ShooterIO {

  private Timer timer = new Timer();

  private PIDController pid = new PIDController(0.1, 0.0, 0.0);

  protected final TalonFXS leadShooter = new TalonFXS(LEAD_SHOOTER_ID);
  protected final TalonFXS followerShooter = new TalonFXS(FOLLOW_SHOOTER_ID);
  protected final TalonFXS turretMotor = new TalonFXS(Turret.TURRET_ID);
  protected final TalonFXS yAxisMotor = new TalonFXS(Turret.Y_AXIS_ID);

  public ShooterIOReal() {
    configureMotor();
    timer.reset();
  }

  private void configureMotor() {
    // Lead configuration
    leadShooter.getConfigurator().apply(CURRENT_LIMIT);
    leadShooter.setNeutralMode(NEUTRAL_MODE);
    leadShooter.getConfigurator().apply(new MotorOutputConfigs().withInverted(LEAD_SHOOTER_INVERT));
    
    // Follower configuration
    followerShooter.getConfigurator().apply(CURRENT_LIMIT);
    followerShooter.setNeutralMode(NEUTRAL_MODE);
    followerShooter.setControl(new Follower(LEAD_SHOOTER_ID, MotorAlignmentValue.Aligned));

    // Turret config
    Slot0Configs turretGain =
      new Slot0Configs().withKP(Turret.TURRET_KP).withKI(Turret.TURRET_KI).withKD(Turret.TURRET_KD);

    turretMotor.getConfigurator().apply(turretGain);


    // Y-Axis config
    Slot0Configs yAxisGain = new Slot0Configs().withKP(Turret.Y_AXIS_KP).withKI(Turret.Y_AXIS_KI).withKD(Turret.Y_AXIS_KD);

    yAxisMotor.getConfigurator().apply(yAxisGain);
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

  @Override
  public void moveTurret(double position) {
    turretMotor.setControl(new MotionMagicVoltage(position));
  }

  @Override
  public void tiltShooter(double position) {
    yAxisMotor.setControl(new MotionMagicVoltage(position));
  }
}
