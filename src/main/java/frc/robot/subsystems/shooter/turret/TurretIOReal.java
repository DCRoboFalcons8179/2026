package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants.C_Shooter.C_Turret;

public class TurretIOReal implements TurretIO {
  protected final TalonFXS turretMotor = new TalonFXS(C_Turret.TURRET_ID);
  
  private double targetPosition = 0.0;
  private boolean usePositionControl = false;
  
  public TurretIOReal() {
    setPIDControl();
  }
  
  @Override
  public void setPIDControl() {
    // Turret config
    Slot0Configs turretGain =
        new Slot0Configs()
            .withKP(C_Turret.TURRET_KP)
            .withKI(C_Turret.TURRET_KI)
            .withKD(C_Turret.TURRET_KD);
    turretMotor.getConfigurator().apply(turretGain);
  }
  
  @Override
  public void stop() {
    turretMotor.set(0);
    turretMotor.stopMotor();
    usePositionControl = false;
    targetPosition = 0.0;
  }
  
  @Override
  public void updateInputs(TurretInputs inputs) {
    inputs.current = turretMotor.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = turretMotor.getPosition().getValueAsDouble();
    inputs.velocity = turretMotor.getVelocity().getValueAsDouble();
    inputs.appliedVoltage = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.targetPosition = targetPosition;
    
    // Check if at target (within tolerance when using position control)
    if (usePositionControl) {
      inputs.atTarget = Math.abs(inputs.encoderPosition - targetPosition) < 0.01; // 0.01 rotations tolerance
    } else {
      inputs.atTarget = false;
    }
  }
  
  @Override
  public void moveTurret(double position) {
    targetPosition = position;
    usePositionControl = true;
    turretMotor.setControl(new MotionMagicVoltage(position));
  }
  
  @Override
  public void moveTurretPO(double omegaPercent) {
    usePositionControl = false;
    turretMotor.setControl(new DutyCycleOut(omegaPercent));
  }
}