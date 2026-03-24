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

  /// for Tags 2, 4, 5, 10, 18, 20, 21, 26
  private static final Translation2d TAG_SET_1 = new Translation2d(0.5842, 0);

  /// for Tags 3, 9, 11, 19, 25, 27
  private static final Translation2d TAG_SET_2 = new Translation2d(0.5842, 0.3556);

  /// for Tags 8, 24
  private static final Translation2d TAG_SET_3 = new Translation2d(0.5842, -0.3556);

  private static final Translation2d BLANK_TRANSLATION = new Translation2d(0, 0);

  public static final Translation2d[] tagsToHub = {
    // 1
    BLANK_TRANSLATION,
    // 2
    TAG_SET_1,
    // 3
    TAG_SET_2,
    // 4
    TAG_SET_1,
    // 5
    TAG_SET_1,
    // 6
    BLANK_TRANSLATION,
    // 7
    BLANK_TRANSLATION,
    // 8
    TAG_SET_3,
    // 9
    TAG_SET_2,
    // 10
    TAG_SET_1,
    // 11
    TAG_SET_2,
    // 12
    BLANK_TRANSLATION,
    // 13
    BLANK_TRANSLATION,
    // 14
    BLANK_TRANSLATION,
    // 15
    BLANK_TRANSLATION,
    // 16
    BLANK_TRANSLATION,
    // 17
    BLANK_TRANSLATION,
    // 18
    TAG_SET_1,
    // 19
    TAG_SET_2,
    // 20
    TAG_SET_1,
    // 21
    TAG_SET_1,
    // 22
    BLANK_TRANSLATION,
    // 23
    BLANK_TRANSLATION,
    // 24
    TAG_SET_3,
    // 25
    TAG_SET_2,
    // 26
    TAG_SET_1,
    // 27
    TAG_SET_2
  };

  public static boolean useCameras = true;
}
