// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// This has to be a command in order to be able to run while disabled. - Gavin
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReachWristBrakeOffCommand extends InstantCommand {
  ReachSubsystem m_reachSubsystem; 
  WristSubsystem m_wristSubsystem;
  boolean m_isEnabledCalled = false;

  public ReachWristBrakeOffCommand(ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem) {
    m_reachSubsystem = reachSubsystem; 
    m_wristSubsystem = wristSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_reachSubsystem, m_wristSubsystem);
  }

  public ReachWristBrakeOffCommand(ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, boolean isEnabledCalled) {
    m_reachSubsystem = reachSubsystem; 
    m_wristSubsystem = wristSubsystem;
    m_isEnabledCalled = isEnabledCalled;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("brakeOffCommand", 1, "initialize");
    boolean reach;
    boolean wrist;
    if (m_isEnabledCalled) {
      reach = true;
      wrist = true;
    } else if (DriverStation.isTeleopEnabled()){
      reach = true;
      wrist = true;
    } else {
      reach = !m_reachSubsystem.getBrakeMode();
      wrist = !m_wristSubsystem.getBrakeMode();
    }
    m_reachSubsystem.setBrakeMode(reach);
    if (Constants.k_isCompetition) {
      m_wristSubsystem.setBrakeMode(wrist);
    }
  }

  @Override 
  public boolean runsWhenDisabled() {
    return true;
  }
}
