package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public static class Shooter {
        /// Shooter ID of Lead Shooter
        public static final int LEAD_SHOOTER_ID = 1;
        /// boolean for the Lead Shooter's invert
        public static final InvertedValue LEAD_SHOOTER_INVERT = InvertedValue.Clockwise_Positive;

        /// Shooter ID of Follower Shooter
        public static final int FOLLOW_SHOOTER_ID = 2;


        /// Current to limit the motors to in amps
        public static final CurrentLimitsConfigs CURRENT_LIMIT = new CurrentLimitsConfigs().withSupplyCurrentLimit(30)
                .withSupplyCurrentLimitEnable(true);

        /// Velocity for the motors to set rot/s (will be calculated in later versions, setting up to get something working rn)
        public static final double OUTPUT_SPEED = 58;

        /// Velocity in rot/s that shooter speed can be under by
        public static final double ERROR_MARGIN = 10;

        /// The mode for the motors when innactive
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    }
}
