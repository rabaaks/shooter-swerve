package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  public static final int feedCanId = 0;
  public static final int shootCanId = 0;

  public static final double shooterAngle = 0.0;

  public static final LoggedTunableNumber kP = new LoggedTunableNumber("/Tuning/kP");
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("/Tuning/kI");
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("/Tuning/kD");
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("/Tuning/kV");
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("/Tuning/kA");
}
