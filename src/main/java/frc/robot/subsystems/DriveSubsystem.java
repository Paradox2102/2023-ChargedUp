// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.ApriltagsCamera.ApriltagsCameraRegion;
import frc.ApriltagsCamera.ApriltagsCamera.ApriltagsCameraRegions;
import frc.robot.LocationTracker;

public class DriveSubsystem extends SubsystemBase {
  public boolean m_goingForward = true;
  ApriltagsCamera m_camera = new ApriltagsCamera();
  Gyro m_gyro = new WPI_PigeonIMU(0);
  LocationTracker m_tracker = new LocationTracker();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  public void setPower(double leftPower, double rightPower) {
    // motor stuff
  }

  public void setBrakeMode(boolean brake){
    //left drive (IdleMode)
    // right drive 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ApriltagsCameraRegions region = m_camera.getRegions();
    //double[] location = m_tracker.getLocation(region.m_targetHorzPos, region.m_targetVertPos, m_gyro.getAngle(), coordinates for the AprilTag here);
    //SmartDashboard.putNumberArray("Robot Location", location);
  }
}
