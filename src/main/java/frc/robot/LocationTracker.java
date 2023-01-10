package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LocationTracker {

    // X and Z are in inches
    // Theta = radians
    // Returns location of the robot from tag in inches
    public Pose2d getLocation(double x, double z, double theta, double[] target) {
        double tx = target[0];
        double ty = target[1];

        double alpha = Math.atan2(x, z);
        double beta = theta - Math.PI/2 - alpha;
        double d = Math.sqrt(x*x + z*z);
        double dx = d * Math.sin(beta);
        double dy = d * Math.cos(beta);
        double robot_x = tx + dx;
        double robot_y = ty - dy;

        return new Pose2d(robot_x, robot_y, new Rotation2d(theta));
      }
    
}
