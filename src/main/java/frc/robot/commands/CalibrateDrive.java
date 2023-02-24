// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Sensor;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrateDrive extends CommandBase {
  private DriveSubsystem m_subsystem;
  private Sensor m_sensor;

  /** Creates a new CalibrateDrive. */
  public CalibrateDrive(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    m_sensor = m_subsystem.getSensors();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("CalibrateDrive", 1, "initialize");
    //m_subsystem.setPower(0.5, 0.5);
    //m_subsystem.setSpeed(4000, 4000);
    m_subsystem.setSpeedFPS(5, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Left velocity", m_sensor.getLeftEncoderVel());
    SmartDashboard.putNumber("Right velocity", m_sensor.getRightEncoderVel());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("CalibrateDrive", 1, "end");
    m_subsystem.setPower(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
