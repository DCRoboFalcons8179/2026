package frc.robot.subsystems.shooter.pitch;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import org.littletonrobotics.junction.Logger;

public class PitchIOReal implements PitchIO {
  protected final TalonFXS pitchMotor = new TalonFXS(PitchConstants.PITCH_ID);

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
            .withKP(PitchConstants.PITCH_KP)
            .withKI(PitchConstants.PITCH_KI)
            .withKD(PitchConstants.PITCH_KD);
    pitchMotor.getConfigurator().apply(pitchGain);
    pitchMotor.setNeutralMode(PitchConstants.PITCH_NEUTRAL_MODE);
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
  public void updateInputs(PitchInputsAutoLogged inputs) {
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

    Logger.processInputs("Pitch", inputs);
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
