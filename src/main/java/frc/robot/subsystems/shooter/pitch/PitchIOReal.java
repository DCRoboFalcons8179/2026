package frc.robot.subsystems.shooter.pitch;

import static frc.robot.Constants.C_Shooter.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.C_Shooter.C_Turret;

public class PitchIOReal implements PitchIO {
  protected final TalonFXS pitchMotor = new TalonFXS(C_Turret.PITCH_ID);

  public PitchIOReal() {
    configureMotor();
    setPIDControl();
  }

  private void configureMotor() {
    // Pitch configuration
    pitchMotor.getConfigurator().apply(CURRENT_LIMIT);
    pitchMotor.setNeutralMode(C_Turret.PITCH_NEUTRAL_MODE);
  }

  @Override
  public void setPIDControl() {
    // Pitch config
    Slot0Configs pitchGain =
        new Slot0Configs()
            .withKP(C_Turret.PITCH_KP)
            .withKI(C_Turret.PITCH_KI)
            .withKD(C_Turret.PITCH_KD);

    pitchMotor.getConfigurator().apply(pitchGain);
  }

  @Override
  public void stop() {
    pitchMotor.stopMotor();
  }

  @Override
  public void updateInputs(PitchInputs inputs) {
    inputs.encoderPosition = pitchMotor.getPosition().getValueAsDouble();
  }

  public void tiltShooter(double position) {
    pitchMotor.setControl(new MotionMagicVoltage(position));
    System.out.println("Pitch Position: " + pitchMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Target Pitch", position);
    SmartDashboard.putNumber("Pitch Position", pitchMotor.getPosition().getValueAsDouble());
  }
}
