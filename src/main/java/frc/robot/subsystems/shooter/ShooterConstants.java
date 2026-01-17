package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  public static final int feedCanId = 0;
  public static final int shootCanId = 0;

  public static final double shooterAngle = 0.0;

  public static final LoggedTunableNumber kP = new LoggedTunableNumber("/Shooter/kP", 1.0);
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("/Shooter/kI", 0.0);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("/Shooter/kD", 0.0);
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("/Shooter/kV", 12/500);
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("/Shooter/kA", 0.0);

  public static final LoggedNetworkNumber velocitySetpoint = new LoggedNetworkNumber("/Tuning/Velocity", 100);
}
