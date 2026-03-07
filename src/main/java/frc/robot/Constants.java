// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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

  ///for Tags 10, 2, 4, 5, 26, 18, 20, 21
  private static final Translation2d TAG_SET_1 = new Translation2d(23.25, 0);
  private static final int[] TAG_SET_1_IDS = {2,4,5,10,18,20,21,26};

  ///for Tags 3, 11, 9, 19, 27, 25
  private static final Translation2d TAG_SET_2 = new Translation2d(14.25, 23.25);
  private static final int[] TAG_SET_2_IDS = {3,9,11,19,25,27};
  /// for Tags 8, 24
  private static final Translation2d TAG_SET_3 = new Translation2d(-14.25, 23.25);
  private static final int[] TAG_SET_3_IDS = {8,24};
}
