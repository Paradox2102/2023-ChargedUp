// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  AprilTagDetector m_detector = new AprilTagDetector();
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  public void setPower(double leftPower, double rightPower) {
    // motor stuff
  }

  public double[] getLocation(double x, double z, double theta, double[] target) {
    double[] location = new double[2];
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

        location[0] = robot_x;
        location[1] = robot_y;

        return location;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
