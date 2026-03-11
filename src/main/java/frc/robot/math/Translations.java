package frc.robot.math;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class Translations {
  public static Translation2d tagToHub(int tagID, Translation2d robotToTag, double yaw) {
    if (tagID == -1 || tagID > 27) {
      System.out.println("Invalid tag ID" + tagID);
      return Translation2d.kZero;
    }

    // Get the translation from the robot to the tag
    double xRT = robotToTag.getX();
    double yRT = robotToTag.getY();

    // Get the translation from the tag to the hub
    double xTH = Constants.tagsToHub[tagID - 1].getX() * Math.sin(yaw);
    double yTH = Constants.tagsToHub[tagID - 1].getY() * Math.cos(yaw);

    // Calculate the translation from the robot to the hub
    double xRH = xRT + xTH;
    double yRH = yRT + yTH;

    return new Translation2d(xRH, yRH);
  }
}
