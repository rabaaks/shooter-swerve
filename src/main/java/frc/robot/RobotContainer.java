// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private Map<RobotState, Trigger> stateRequests = new EnumMap<>(RobotState.class);
  private Map<RobotState, Trigger> stateTriggers = new EnumMap<>(RobotState.class);

  @AutoLogOutput(key = "Robot State/Current State")
  private RobotState state = RobotState.IDLE;

  @AutoLogOutput(key = "Robot State/Previous State")
  private RobotState previousState = RobotState.IDLE;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO[] {
                  new VisionIOPhotonVision("camera0", robotToCamera0),
                  new VisionIOPhotonVision("camera1", robotToCamera1)
                });
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO[] {
                  new VisionIOPhotonVisionSim("camera0", robotToCamera0, drive::getPose),
                  new VisionIOPhotonVision("camera1", robotToCamera1)
                });
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO[] {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureStates();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    stateRequests.put(RobotState.IDLE, controller.y());
    stateRequests.put(RobotState.INTAKE, controller.leftTrigger());
    stateRequests.put(RobotState.PRESCORE, controller.rightBumper());
    stateRequests.put(RobotState.SCORE, controller.rightTrigger());

    for (RobotState state : RobotState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }
  }

  private void configureStates() {
    // Transitions
    stateTriggers
        .get(RobotState.INTAKE)
        .or(stateTriggers.get(RobotState.PRESCORE))
        .and(stateRequests.get(RobotState.IDLE))
        .onTrue(forceState(RobotState.IDLE));

    stateTriggers
        .get(RobotState.IDLE)
        .and(stateRequests.get(RobotState.INTAKE))
        .onTrue(forceState(RobotState.INTAKE));

    stateTriggers
        .get(RobotState.IDLE)
        .or(stateTriggers.get(RobotState.INTAKE))
        .and(stateRequests.get(RobotState.PRESCORE))
        .onTrue(forceState(RobotState.PRESCORE));

    stateTriggers
        .get(RobotState.PRESCORE)
        .and(stateRequests.get(RobotState.SCORE))
        .onTrue(forceState(RobotState.SCORE));

    stateTriggers
        .get(RobotState.SCORE)
        .and(stateRequests.get(RobotState.SCORE))
        .onFalse(forceState(RobotState.PRESCORE));

    // Normal arcade drive
    stateTriggers
        .get(RobotState.IDLE)
        .or(stateTriggers.get(RobotState.INTAKE))
        .onTrue(
            Commands.parallel(
                Commands.run(() -> System.out.println("idl")),
                Commands.runOnce(shooter::stopFeed),
                Commands.runOnce(shooter::stopShoot),
                DriveCommands.joystickDrive(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> -controller.getRightX())));

    // Lock onto target
    // Drive gets angle to target, shooter gets distance and height (delta x and delta y)
    stateTriggers
        .get(RobotState.PRESCORE)
        .or(stateTriggers.get(RobotState.SCORE))
        .onTrue(
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getRightX(),
                    () ->
                        targetFieldTranslation.minus(drive.getPose().getTranslation()).getAngle()),
                Commands.run(
                    () ->
                        shooter.setTarget(
                            new Translation2d(
                                drive
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(targetFieldTranslation),
                                targetZ)))));

    stateTriggers.get(RobotState.PRESCORE).onTrue(Commands.runOnce(shooter::stopFeed));

    stateTriggers.get(RobotState.SCORE).onTrue(Commands.runOnce(shooter::feed));
  }

  private Command forceState(RobotState nextState) {
    return Commands.runOnce(
        () -> {
          System.out.println("Changing state to " + nextState);
          previousState = state;
          state = nextState;
        });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
