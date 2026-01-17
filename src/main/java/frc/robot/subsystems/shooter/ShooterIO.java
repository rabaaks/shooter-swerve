package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shootVelocity = 0.0;
    public double shootAppliedVolts = 0.0;
    public double[] shootCurrent = new double[] {};

    public double feedAppliedVolts = 0.0;
    public double[] feedCurrent = new double[] {};
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShootVelocity(double velocity) {}

  public default void setBallVelocity(double velocity) {}

  public default void setShootVoltage(double voltage) {}

  public default void setFeedVoltage(double voltage) {}
}
