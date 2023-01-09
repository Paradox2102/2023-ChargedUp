package frc.robot;

public class LocationTracker {

    public double[] getLocation(double x, double z, double theta, double[] target) {
        double toRadian = Math.PI/180;
        double[] location = new double[2];
        double tx = target[0];
        double ty = target[1];
        for (double i = theta; i >= 360; i = theta){
            theta -= 360;
        }
        theta *= toRadian;

        double alpha = Math.atan(x/z);
        double beta = theta - 90 - alpha;
        double d = Math.sqrt(x*x + z*z);
        double dx = d * Math.sin(beta);
        double dy = d * Math.cos(beta);
        double robot_x = tx + dx;
        double robot_y = ty - dy;

        location[0] = robot_x;
        location[1] = robot_y;

        return location;
      }
    
}
