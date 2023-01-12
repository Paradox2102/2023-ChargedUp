// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.ApriltagsCamera.ApriltagsCameraRegion;
import frc.ApriltagsCamera.ApriltagsCamera.ApriltagsCameraRegions;
import frc.robot.Constants;
import frc.robot.LocationTracker;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  ApriltagsCamera m_camera;
  Gyro m_gyro = new WPI_PigeonIMU(0);
  LocationTracker m_tracker = new LocationTracker();
  private final Field2d m_field = new Field2d();
  AprilTagFieldLayout m_tags;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(ApriltagsCamera camera, AprilTagFieldLayout tags) {
    SmartDashboard.putData("Field", m_field);
    m_camera = camera;
    m_tags = tags;
  }

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
    ApriltagsCameraRegions regions = m_camera.getRegions();
    if (regions != null && regions.getRegionCount() >= 1) {
      ApriltagsCameraRegion region = regions.getRegion(0);
      Pose2d location = m_tracker.getLocation(region.m_tvec[0], region.m_tvec[2], m_gyro.getAngle(), m_tracker.getTag(region.m_tag, m_tags));
      m_field.setRobotPose(location);
      
    } 
    SmartDashboard.putNumber("Region Count", m_camera.getRegions().getRegionCount());
  }
}
