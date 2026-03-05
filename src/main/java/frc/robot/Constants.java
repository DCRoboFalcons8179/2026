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

  public static class C_Shooter {
    /// ID of Lead Shooter
    public static final int LEAD_SHOOTER_ID = 4;
    /// boolean for the Lead Shooter's invert
    public static final InvertedValue LEAD_SHOOTER_INVERT = InvertedValue.CounterClockwise_Positive;

    /// KP for Lead Shooter
    public static final double SHOOTER_KP = 1;
    /// KI for Lead Shooter
    public static final double SHOOTER_KI = 0;
    /// KD for Lead Shooter
    public static final double SHOOTER_KD = 0;
    /// KV for Shooter
    public static final double SHOOTER_KV = 0.2;

    /// ID of Follower Shooter
    // public static final int FOLLOW_SHOOTER_ID = 2;

    /// Current to limit the motors to in amps
    public static final CurrentLimitsConfigs CURRENT_LIMIT =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

    private static final double GEAR_RATIO = 1.0 / 2.0;

    /// Velocity for the motors to set rot/s (will be calculated in later versions,
    /// setting up to get something working rn)
    /// Multiply by 1/GEAR_RATIO to convert from output shaft speed to motor speed
    public static final double OUTPUT_SPEED = 50 * 1 / GEAR_RATIO;

    /// Velocity in rot/s that shooter speed can be under by
    public static final double ERROR_MARGIN = 10;

    /// The mode for the motors when innactive
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

    /// ID of feed motor
    public static final int SHOOT_FEED_ID = 2;

    public static final InvertedValue SHOOT_FEED_INVERT = InvertedValue.CounterClockwise_Positive;

    /// KP for Shoot feeder
    public static final double SHOOT_FEED_KP = 1.3;
    /// KI for Shoot feeder
    public static final double SHOOT_FEED_KI = 0;
    /// KD for Shoot feeder
    public static final double SHOOT_FEED_KD = 0;
    /// Current to limit the motors to in amps
    public static final CurrentLimitsConfigs SHOOT_FEED_CURRENT_LIMIT =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

    private static final double SHOOT_FEED_GEAR_RATIO = 1.0 / 5.0;

    /// Velocity for the motors to set rot/s (will be calculated in later versions,
    /// setting up to get something working rn)
    /// Multiply by 1/GEAR_RATIO to convert from output shaft speed to motor speed
    public static final double SHOOT_FEED_OUTPUT_SPEED = 15 * 1 / SHOOT_FEED_GEAR_RATIO;

    public static class C_Turret {
      /// ID of the turret motor
      public static final int TURRET_ID = 3;

      /// The mode for the turret motor when inactive
      public static final NeutralModeValue TURRET_NEUTRAL_MODE = NeutralModeValue.Coast;

      /// KP for moving to pos
      public static final double TURRET_KP = 1;
      /// KI for moving to pos
      public static final double TURRET_KI = 0;
      /// KD for moving to pos
      public static final double TURRET_KD = 0;

      /// KP for auto aim
      public static final double AIM_KP = 0;
      /// KI for auto aim
      public static final double AIM_KI = 0;
      /// KD for auto aim
      public static final double AIM_KD = 0;

      /// ID of the y-axis controlling motor
      public static final int PITCH_ID = 5;

      public static final double PITCH_KP = 1;
      public static final double PITCH_KI = 0;
      public static final double PITCH_KD = 0;

      public static final NeutralModeValue PITCH_NEUTRAL_MODE = NeutralModeValue.Brake;
    }
  }

  public static class Intake {
    // ID for feeder
    public static final int FEEDER_ID = 6;

    public static final double GEAR_RATIO = 1.0 / 3.0;

    // motor velocity (current just a placeholder number before being tested)
    public static final double FEEDER_SPEED_IN = -18.5;
    public static final double FEEDER_SPEED_OUT = 18.5;

    // Neutral Mode for feeder
    public static final NeutralModeValue FEEDER_NEUTRAL_MODE = NeutralModeValue.Coast;

    public static final CurrentLimitsConfigs FEEDER_CURRENT_LIMIT =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);
  }

  public static class Extruder {
    // ID for extruder
    public static final int EXTRUDER_ID = 7;

    // Neutral Mode for Extruder
    public static final NeutralModeValue EXTRUDER_NEUTRAL_MODE = NeutralModeValue.Brake;

    public static final CurrentLimitsConfigs EXTRUDER_CURRENT_LIMIT =
        new CurrentLimitsConfigs().withSupplyCurrentLimit(30).withSupplyCurrentLimitEnable(true);

    public static final double EXTRUDER_IN_POSITION = 0;
    public static final double EXTRUDER_OUT_POSITION = -33;
    public static final double EXTRUDER_MANAL_DELTA = -3;
    // Error thresh hold (current just a placeholder number before being tested)
    public static final double EXTRUDER_ERROR_THRESH_HOLD = .2;

    // PID values for extruder - These need tuning
    public static final double EXTRUDER_KP = 26;
    public static final double EXTRUDER_KI = 0;
    public static final double EXTRUDER_KD = 0.1;

    // Motion profiling constraints to control speed
    // Maximum velocity in rotations per second (adjust lower to slow down)
    public static final double EXTRUDER_MAX_VELOCITY = 48.0; // 24 rotations/sec - adjust as needed
    // Maximum acceleration in rotations per second squared
    public static final double EXTRUDER_MAX_ACCELERATION = 21.0; // Smooth acceleration
    // Jerk control for even smoother motion (rotations per second cubed)
    public static final double EXTRUDER_JERK = 40.0; // Optional smoothing
  }
}
