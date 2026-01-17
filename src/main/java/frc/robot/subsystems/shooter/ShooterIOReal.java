package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class ShooterIOReal implements ShooterIO {
  private final TalonSRX feedMotor = new TalonSRX(feedCanId);
  private final TalonSRX shootMotor = new TalonSRX(shootCanId);

  private double shootVelocityTarget = 0.0;

  public ShooterIOReal() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.voltageCompSaturation = 12.0;

    feedMotor.configAllSettings(config);
    shootMotor.configAllSettings(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shootVelocity = shootVelocityTarget;
    inputs.shootAppliedVolts = shootMotor.getMotorOutputVoltage();
    inputs.shootCurrent = new double[] {shootMotor.getStatorCurrent()};
    inputs.feedAppliedVolts = feedMotor.getMotorOutputVoltage();
    inputs.feedCurrent = new double[] {feedMotor.getStatorCurrent()};
  }

  @Override
  public void setShootVelocity(double velocity) {
    shootMotor.set(TalonSRXControlMode.PercentOutput, velocity * 0.0);
  }

  @Override
  public void setShootVoltage(double voltage) {
    shootMotor.set(TalonSRXControlMode.PercentOutput, voltage / 12.0);
  }

  @Override
  public void setFeedVoltage(double voltage) {
    feedMotor.set(TalonSRXControlMode.PercentOutput, voltage / 12.0);
  }
}
