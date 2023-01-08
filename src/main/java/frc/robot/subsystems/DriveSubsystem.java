// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  public boolean m_goingForward = true;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  public void setPower(double leftPower, double rightPower) {
    // motor stuff
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
