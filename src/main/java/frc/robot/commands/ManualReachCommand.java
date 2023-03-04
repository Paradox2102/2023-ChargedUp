// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ReachSubsystem;

public class ManualReachCommand extends CommandBase {
  ReachSubsystem m_subsystem;
  double m_power;
  boolean m_runP;
  /** Creates a new ManualReachCommand. */
  public ManualReachCommand(ReachSubsystem reachSubsystem, double power, boolean runP) {
    m_subsystem = reachSubsystem;
    m_power = power;
    m_runP = runP;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ManualReachCommand", 1, "initialize");
    m_subsystem.setPower(m_power);
    m_subsystem.isRunP(m_runP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("ManualReachCommand", 1, "end");
    m_subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
