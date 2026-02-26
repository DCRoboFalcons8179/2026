package frc.robot.subsystems.shooter.pitch;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.C_Shooter.C_Turret;

public class PitchIOReal implements PitchIO {
  protected final TalonFXS pitchMotor = new TalonFXS(C_Turret.PITCH_ID);

  private double targetPosition = 0.0;
  private boolean usePositionControl = false;

  public PitchIOReal() {
    setPIDControl();
  }

  @Override
  public void setPIDControl() {
    // Pitch PID config
    Slot0Configs pitchGain =
        new Slot0Configs()
            .withKP(C_Turret.PITCH_KP)
            .withKI(C_Turret.PITCH_KI)
            .withKD(C_Turret.PITCH_KD);
    pitchMotor.getConfigurator().apply(pitchGain);

    // Motion Magic configuration - required for MotionMagicVoltage to work
    MotionMagicConfigs mmConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(80) // rotations per second
            .withMotionMagicAcceleration(160) // rotations per second squared
            .withMotionMagicJerk(1600); // rotations per second cubed
    pitchMotor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public void stop() {
    pitchMotor.set(0);
    pitchMotor.stopMotor();
    usePositionControl = false;
    targetPosition = 0.0;
  }

  @Override
  public void updateInputs(PitchInputs inputs) {
    inputs.current = pitchMotor.getTorqueCurrent().getValueAsDouble();
    inputs.encoderPosition = pitchMotor.getPosition().getValueAsDouble();
    inputs.velocity = pitchMotor.getVelocity().getValueAsDouble();
    inputs.appliedVoltage = pitchMotor.getMotorVoltage().getValueAsDouble();
    inputs.targetPosition = targetPosition;

    // Check if at target (within tolerance when using position control)
    if (usePositionControl) {
      inputs.atTarget =
          Math.abs(inputs.encoderPosition - targetPosition) < 0.01; // 0.01 rotations tolerance
    } else {
      inputs.atTarget = false;
    }

    // Add debugging info to SmartDashboard
    SmartDashboard.putNumber("Pitch Current Position", inputs.encoderPosition);
    SmartDashboard.putNumber("Pitch Current (A)", inputs.current);
    SmartDashboard.putNumber("Pitch Velocity", inputs.velocity);
    SmartDashboard.putNumber("Pitch Applied Voltage", inputs.appliedVoltage);
  }

  @Override
  public void movePitch(double position) {
    targetPosition = position;
    usePositionControl = true;
    pitchMotor.setControl(new MotionMagicVoltage(position));
  }

  @Override
  public double getPitchPosition() {
    return pitchMotor.getPosition().getValueAsDouble();
  }
}
