package frc.robot.math;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class Translations {
    public static Translation2d tagToHub(int tagID, Translation2d robotToTag) {
        // Get the translation from the robot to the tag
        double xRT = robotToTag.getX();
        double yRT = robotToTag.getY();

        // Get the translation from the tag to the hub
        double xTH = Constants.tagsToHubs[tagID - 1].getX();
        double yTH = Constants.tagsToHubs[tagID - 1].getY();

        // Calculate the translation from the robot to the hub
        double xRH = xRT + xTH;
        double yRH = yRT + yTH;

        return new Translation2d(xRH, yRH);
    }
}
