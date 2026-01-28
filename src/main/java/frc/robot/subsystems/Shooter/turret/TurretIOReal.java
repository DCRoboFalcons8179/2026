package frc.robot.subsystems.Shooter.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants.Shooter.Turret;

public class TurretIOReal implements TurretIO {

  protected final TalonFXS turretMotor = new TalonFXS(Turret.TURRET_ID);

  public TurretIOReal() {
    setPIDControl();
  }

  @Override
  public void setPIDControl() {
    // Turret config
    Slot0Configs turretGain =
        new Slot0Configs()
            .withKP(Turret.TURRET_KP)
            .withKI(Turret.TURRET_KI)
            .withKD(Turret.TURRET_KD);

    turretMotor.getConfigurator().apply(turretGain);
  }

  @Override
  public void stop() {
    turretMotor.set(0);
    turretMotor.stopMotor();
  }

  @Override
  public void updateInputs(TurretInputs inputs) {
    inputs.current = turretMotor.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = turretMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void moveTurret(double position) {
    turretMotor.setControl(new MotionMagicVoltage(position));
  }
}
