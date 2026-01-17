// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class ShooterConstants {
    public static final double SIM_TOLERANCE = 0.5;

    // used in Shooter.java
    public static final double RAMP_RATE_VOLTS_SYSID = 0.25;
    public static final double DYNAMIC_STEP_VOLTS_SYSID = 1;
    public static final double TOLERANCE = 0.1;
  }

  public class TargetPoses {
    public static final Pose2d HUB_RED_POSE2D = new Pose2d(11.864594, 4.034536, new Rotation2d());
    public static final Pose2d HUB_BLUE_POSE2D = new Pose2d(4.574794, 4.034536, new Rotation2d());
  }
}
