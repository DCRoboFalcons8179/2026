package frc.robot.subsystems.shooter;

import static frc.robot.Constants.C_Shooter.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.C_Shooter.C_Turret;

public class ShooterIOReal implements ShooterIO {

  private Timer timer = new Timer();

  private PIDController pid = new PIDController(0.1, 0.0, 0.0);

  protected final TalonFXS leadShooter = new TalonFXS(LEAD_SHOOTER_ID);
  protected final TalonFXS followerShooter = new TalonFXS(FOLLOW_SHOOTER_ID);
  protected final TalonFXS turretMotor = new TalonFXS(C_Turret.TURRET_ID);
  protected final TalonFXS yAxisMotor = new TalonFXS(C_Turret.Y_AXIS_ID);

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

    // Y-Axis config
    Slot0Configs yAxisGain =
        new Slot0Configs()
            .withKP(C_Turret.Y_AXIS_KP)
            .withKI(C_Turret.Y_AXIS_KI)
            .withKD(C_Turret.Y_AXIS_KD);

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
}
