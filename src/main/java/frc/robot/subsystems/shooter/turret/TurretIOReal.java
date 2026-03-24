package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.subsystems.Music;
import org.littletonrobotics.junction.Logger;

public class TurretIOReal implements TurretIO {
  protected final TalonFXS turretMotor = new TalonFXS(TurretConstants.TURRET_ID);

  private double targetPosition = 0.0;
  private boolean usePositionControl = false;

  public TurretIOReal() {
    setPIDControl();

    Music.addMotor(turretMotor);
  }

  @Override
  public void setPIDControl() {
    // Turret PID config
    Slot0Configs turretGain =
        new Slot0Configs()
            .withKP(TurretConstants.TURRET_KP)
            .withKI(TurretConstants.TURRET_KI)
            .withKD(TurretConstants.TURRET_KD);
    turretMotor.getConfigurator().apply(turretGain);

    turretMotor.setPosition(0);

    // Motion Magic configuration - required for MotionMagicVoltage to work
    MotionMagicConfigs mmConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(80) // rotations per second
            .withMotionMagicAcceleration(160) // rotations per second squared
            .withMotionMagicJerk(1600); // rotations per second cubed
    turretMotor.getConfigurator().apply(mmConfigs);

    turretMotor.setNeutralMode(TurretConstants.TURRET_NEUTRAL_MODE);
  }

  @Override
  public void stop() {
    turretMotor.set(0);
    turretMotor.stopMotor();
    usePositionControl = false;
    targetPosition = 0.0;
  }

  @Override
  public void updateInputs(TurretInputsAutoLogged inputs) {
    inputs.current = turretMotor.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = turretMotor.getPosition().getValueAsDouble();
    inputs.velocity = turretMotor.getVelocity().getValueAsDouble();
    inputs.appliedVoltage = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.targetPosition = targetPosition;

    // Check if at target (within tolerance when using position control)
    if (usePositionControl) {
      inputs.atTarget =
          Math.abs(inputs.encoderPosition - targetPosition) < 0.01; // 0.01 rotations tolerance
    } else {
      inputs.atTarget = false;
    }

    Logger.processInputs("Turret", inputs);
  }

  @Override
  public void moveTurret(double position) {
    position += TurretConstants.NUDGE_AMOUNT;

    if (position > TurretConstants.MAX_MOTOR_ROT) {
      targetPosition = TurretConstants.MAX_MOTOR_ROT;
    } else if (position < TurretConstants.MIN_MOTOR_ROT) {
      targetPosition = TurretConstants.MIN_MOTOR_ROT;
    } else {
      targetPosition = position;
    }

    usePositionControl = true;
    turretMotor.setControl(new MotionMagicVoltage(targetPosition));
  }

  @Override
  public void incrementTurret(double increment) {
    moveTurret(targetPosition + increment);
  }

  @Override
  public double getTurretPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }
}
