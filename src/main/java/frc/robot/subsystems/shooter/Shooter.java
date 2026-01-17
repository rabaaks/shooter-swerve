package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setTarget(Translation2d target) {
    double x = target.getX();
    double y = target.getY();
    double g = 9.8;
    // double theta = Math.atan((y + Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))) / x);
    double theta = shooterAngle;
    double v0 =
        Math.sqrt(
            g * Math.pow(x, 2) / (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y)));

    io.setShootVelocity(v0);
  }

  public void feed() {
    io.setFeedVoltage(6.0);
  }

  public void stopFeed() {
    io.setFeedVoltage(0.0);
  }

  public void stopShoot() {
    io.setShootVoltage(0.0);
  }
}
