// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  public static class Shooter {
    /// Shooter ID of Lead Shooter
    public static final int LEAD_SHOOTER_ID = 1;
    /// boolean for the Lead Shooter's invert
    public static final InvertedValue LEAD_SHOOTER_INVERT = InvertedValue.Clockwise_Positive;

    /// Shooter ID of Follower Shooter
    public static final int FOLLOW_SHOOTER_ID = 2;

    /// Current to limit the motors to in amps
    public static final CurrentLimitsConfigs CURRENT_LIMIT =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

    /// Velocity for the motors to set rot/s (will be calculated in later versions,
    /// setting up to get something working rn)
    public static final double OUTPUT_SPEED = 58;

    /// Velocity in rot/s that shooter speed can be under by
    public static final double ERROR_MARGIN = 10;

    /// The mode for the motors when innactive
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
  }

  public static class Intake {
    // ID for feeder
    public static final int FEEDER_ID = 1;
    // ID for extruder
    public static final int EXTRUDER_ID = 0;

    // motor velocity (current just a placeholder number before being tested)
    public static final double FEEDER_SPEED_IN = 10;
    public static final double FEEDER_SPEED_OUT = -10;

    // Neutral Mode for feeder
    public static final NeutralModeValue FEEDER_NEUTRAL_MODE = NeutralModeValue.Coast;
    // Neutral Mode for Extruder
    public static final NeutralModeValue EXTRUDER_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final double EXTRUDER_IN_POSITION = 0;

    public static final double EXTRUDER_OUT_POSITION = 5;
    // Error thresh hold (current just a placeholder number before being tested)
    public static final double EXTRUDER_ERROR_THRESH_HOLD = .5;
  }
}
