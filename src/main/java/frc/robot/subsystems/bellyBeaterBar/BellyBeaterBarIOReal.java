package frc.robot.subsystems.bellyBeaterBar;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.subsystems.Music;
import org.littletonrobotics.junction.Logger;

public class BellyBeaterBarIOReal implements BellyBeaterBarIO {
  protected final TalonFXS bBB = new TalonFXS(BellyBeaterBarConstants.ID);

  protected double percentOut = 0;

  public BellyBeaterBarIOReal() {
    configureMotor();

    Music.addMotor(bBB);
  }

  private void configureMotor() {
    // Lead configuration
    bBB.setNeutralMode(BellyBeaterBarConstants.NEUTRAL_MODE_VALUE);
    bBB.getConfigurator().apply(new MotorOutputConfigs().withInverted(BellyBeaterBarConstants.INVERT));
  }

  @Override
  public void inBBB() {
    percentOut = BellyBeaterBarConstants.IN_SPEED;
    bBB.set(BellyBeaterBarConstants.IN_SPEED);
  }

  @Override
  public void outBBB() {
    percentOut = BellyBeaterBarConstants.OUT_SPEED;
    bBB.set(BellyBeaterBarConstants.OUT_SPEED);
  }

  @Override
  public void stop() {
    bBB.set(0);
    bBB.stopMotor();
  }


  @Override
  public void updateInputs(BellyBeaterBarInputsAutoLogged inputs) {
    inputs.current = bBB.getTorqueCurrent().getValueAsDouble();
    inputs.appliedVoltage = bBB.getMotorVoltage().getValueAsDouble();
    inputs.percentOut = percentOut;

    Logger.processInputs("BellyBeaterBar", inputs);
  }
}
