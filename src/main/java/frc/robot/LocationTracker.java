package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LocationTracker {

    public Pose2d getLocation(double x, double z, double theta, double[] target) {
        double tx = target[0];
        double ty = target[1];
        for (double i = theta; i >= 360; i = theta){
            theta -= 360;
        }

        double alpha = Math.atan(x/z);
        double beta = theta - 90 - alpha;
        double d = Math.sqrt(x*x + z*z);
        double dx = d * Math.sin(beta);
        double dy = d * Math.cos(beta);
        double robot_x = tx + dx;
        double robot_y = ty - dy;

        return new Pose2d(robot_x, robot_y, Rotation2d.fromDegrees(theta));
      }
    
}
