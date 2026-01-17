// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.kA;
import static frc.robot.subsystems.shooter.ShooterConstants.kD;
import static frc.robot.subsystems.shooter.ShooterConstants.kI;
import static frc.robot.subsystems.shooter.ShooterConstants.kP;
import static frc.robot.subsystems.shooter.ShooterConstants.kV;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSparkMax implements ShooterIO {
  private final SparkMax shooter;
  private final SparkMax feeder;

  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;

  private SparkMaxConfig config;

  public ShooterIOSparkMax(int shootID, int feederID) {
    shooter = new SparkMax(shootID, MotorType.kBrushless);
    feeder = new SparkMax(feederID, MotorType.kBrushed);

    config.inverted(true);
    config.smartCurrentLimit(100);
    config.voltageCompensation(12.0);

    config.encoder.velocityConversionFactor((2 * Math.PI) / 60);
    config.encoder.positionConversionFactor(2 * Math.PI);

    config.closedLoop.feedForward.kV(ShooterConstants.kV.getAsDouble());
    config.closedLoop.feedForward.kA(ShooterConstants.kA.getAsDouble());
    config.closedLoop.pid(
        ShooterConstants.kP.getAsDouble(),
        ShooterConstants.kI.getAsDouble(),
        ShooterConstants.kD.getAsDouble());

    controller = shooter.getClosedLoopController();
    encoder = shooter.getEncoder();

    shooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.feedAppliedVolts = feeder.getAppliedOutput() * feeder.getBusVoltage();
    inputs.feedCurrent = new double[] {feeder.getAppliedOutput()};

    inputs.shootAppliedVolts = shooter.getAppliedOutput() * shooter.getBusVoltage();
    inputs.shootCurrent = new double[] {shooter.getAppliedOutput()};

    inputs.shootVelocity = encoder.getVelocity();

    if (kP.hasChanged(hashCode())) {
      this.updatePID();
    }

    if (kI.hasChanged(hashCode())) {
      this.updatePID();
    }

    if (kD.hasChanged(hashCode())) {
      this.updatePID();
    }

    if (kV.hasChanged(hashCode())) {
      this.updatePID();
    }

    if (kA.hasChanged(hashCode())) {
      this.updatePID();
    }
  }

  public void updatePID() {
    config.closedLoop.pid(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
    config.closedLoop.feedForward.kA(kA.getAsDouble());
    config.closedLoop.feedForward.kV(kV.getAsDouble());

    shooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setShootVelocity(double velocity) {
    controller.setSetpoint(velocity, ControlType.kVelocity);
  }

  @Override
  public void setFeedVoltage(double voltage) {
    feeder.setVoltage(voltage);
  }

  @Override
  public void setShootVoltage(double voltage) {
    shooter.setVoltage(voltage);
  }
}
