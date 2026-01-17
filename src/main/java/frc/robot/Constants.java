// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final Translation3d targetTranslation = new Translation3d(5, 5, 5);
  public static final Translation2d targetFieldTranslation = new Translation2d(targetTranslation.getX(),
      targetTranslation.getY());
  public static final double targetZ = targetTranslation.getZ();

  public static final boolean tuningMode = true;

  public static Pose2d hubCenter = new Pose2d(VisionConstants.fieldLayout.getTagPose(18).get().getX(),
      VisionConstants.fieldLayout.getTagPose(26).get().getY(), new Rotation2d());

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotState {
    IDLE,
    INTAKE,
    PRESCORE,
    SCORE
  }
}
