package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.turret.TurretConstants;

public class DriveByCalcs {
    private static double numerator = 0;
    private static double denominator = 0;

    public static double getNumerator() {
        return numerator;
    }

    public static double getDenominator() {
        return denominator;
    }
    
    /**
     * Get the numerator and denomitor to be used for calculating the turret angle and ball launch velocity
     * @param robotPose Position of the robot on the field
     * @param robotVr Robot Velocity relative to robot
     * @param robotAngle Robot angle to the ground
     * @param robotRotationsPerSecond Robot Rotations/s (relative to all reference frames)
     * @return void
     */
    public static void updateDriveByValues(Pose2d robotPose, Translation2d robotVr, Rotation2d robotAngle, double robotRotationsPerSecond) {
        // Pose2d robotPose = drive.getPose();

        Pose3d hubPose = new Pose3d(
                FieldConstants.getTargetData(FieldConstants.HUB_POSITION).getX(),
                FieldConstants.getTargetData(FieldConstants.HUB_POSITION).getY(),
                FieldConstants.HUB_HEIGHT,
                Rotation3d.kZero);

        // Robot Velocity relative to robot
        // Translation2d robotVr = drive.getRobotVelocityComponents
        // Robot angle to the ground
        // Rotation2d robotAngle = drive.getRotation();

        // Robot Velocity relative to ground
        Translation2d robotVc = robotVr.rotateBy(robotAngle);
        // Dereference object of robot velocity relative to robot
        robotVr = null;

        // Robot Rotations/s (relative to all reference frames)
        // double robotRotationsPerSecond = drive.getAngularVelocity();

        // Distance from robot to hub
        Translation2d delta = FieldConstants.getTargetData(FieldConstants.HUB_POSITION)
                .minus(robotPose.getTranslation().plus(TurretConstants.TURRET_POSE));

        // Distance from current pose to the hub
        double length = delta.getNorm();

        // Change in height from turret to hub
        double deltaHeight = FieldConstants.HUB_HEIGHT - TurretConstants.TURRET_HEIHGT;

        // Velocity of the ball relative to the ground
        double ballVg = length * Math.sqrt(
                (Constants.g) / (2 * Math.toDegrees(Math.tan(TurretConstants.PITCH)) * length - deltaHeight));

        // Ratio of the robot to the hub (sin of phi)
        double robotToHubRatioSin = (hubPose.getX() - robotPose.getX()) / length;
        double robotToHubRatioCos = (hubPose.getY() - robotPose.getY()) / length;

        double robotAngleCos = robotAngle.getCos();
        double robotAngleSin = robotAngle.getSin();

        // Turret ratio numerator before angle computation
        double turretRatioNumerator = ballVg * robotToHubRatioSin - robotVc.getX()
                + robotRotationsPerSecond * TurretConstants.TURRET_POSE.getX() * robotAngleSin
                + robotRotationsPerSecond * TurretConstants.TURRET_POSE.getY() * robotAngleCos;

        // Turret ratio denominator before angle computation
        double turretRationDenominator = -ballVg * robotToHubRatioCos - robotVc.getY()
                - robotRotationsPerSecond * TurretConstants.TURRET_POSE.getX() * robotAngleCos
                + robotRotationsPerSecond * TurretConstants.TURRET_POSE.getY() * robotAngleSin;

        numerator = turretRatioNumerator;
        denominator = turretRationDenominator;
        
        // Turret angle in degrees
        // double turretAngle = Math.toDegrees(Math.atan(turretRatioNumerator / turretRationDenominator));
    }
}
