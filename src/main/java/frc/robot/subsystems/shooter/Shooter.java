package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void setTarget(Translation2d target) {
    double x = target.getX();
    double y = target.getY();
    double g = 9.8;
    // double theta = Math.atan((y + Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))) /
    // x);
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

  public double getShooterVoltage() {
    return inputs.shootAppliedVolts;
  }

  public AngularVelocity getShooterVelocity() {
    return AngularVelocity.ofBaseUnits(inputs.shootVelocity, RadiansPerSecond);
  }

  public Command sysID() {
    return Commands.sequence(
        this.getShooterSysIdQuasistatic(Direction.kForward),
        this.getShooterSysIdQuasistatic(Direction.kReverse),
        this.getShooterSysIdDynamic(Direction.kForward),
        this.getShooterSysIdDynamic(Direction.kReverse));
  }

  public Command getShooterSysIdQuasistatic(Direction direction) {
    return new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> io.setShootVoltage(volts.in(Volts)),
                log -> {
                  log.motor("shooter")
                      .voltage(Voltage.ofBaseUnits(this.getShooterVoltage(), Volts))
                      .angularVelocity(this.getShooterVelocity());
                },
                this))
        .quasistatic(direction);
  }

  public Command getShooterSysIdDynamic(Direction direction) {
    return new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> io.setShootVoltage(volts.in(Volts)),
                log -> {
                  log.motor("shooter")
                      .voltage(Voltage.ofBaseUnits(this.getShooterVoltage(), Volts))
                      .angularVelocity(this.getShooterVelocity());
                },
                this))
        .dynamic(direction);
  }
}
