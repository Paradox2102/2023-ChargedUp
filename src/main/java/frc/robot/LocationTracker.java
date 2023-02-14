package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ApriltagsCamera.Logger;

public class LocationTracker {

  // X and Z are in inches
  // Theta = radians
  // Returns location of the robot from tag in inches
  public Pose2d getLocation(double x, double z, double theta, double[] target) {
    double tx = target[0];
    double ty = target[1];

    //theta = 0;
    double alpha = Math.atan2(x, z);
    double beta = theta - Math.PI/2 - alpha;
    double d = Math.sqrt(x*x + z*z);
    double dx = d * Math.sin(beta);
    double dy = d * Math.cos(beta);
    double robot_x = tx + dx;
    double robot_y = ty - dy;
    SmartDashboard.putNumber("x", robot_x);
    SmartDashboard.putNumber("y", robot_y);

    return new Pose2d(robot_x, robot_y, new Rotation2d(theta));
  }

  public double[] getTag(int tagID, AprilTagFieldLayout tags) {
    //System.out.println(String.format("tag=%d", tagID));
    Optional<Pose3d> optionalTag = tags.getTagPose(tagID);
    if (optionalTag.isPresent()) {
      double AprilTag[] = {optionalTag.get().getX(), optionalTag.get().getY()};
      return AprilTag;
    } else {
      Logger.log("LocationTracker", 1, String.format("Tag does not exist %d", tagID));
    }

    return null;
  }
  
    
}
