// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ReachSubsystem;

public class SetArmExtent extends CommandBase {
  private ReachSubsystem m_subsystem;
  private double m_extentInInches;

  private static final double k_p = 0.1;
  private static final double k_deadZone = 1;

  /** Creates a new SetArmExtent. */
  public SetArmExtent(ReachSubsystem subsystem, double extentInInches) {
    m_subsystem = subsystem;
    m_extentInInches = extentInInches;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = m_subsystem.getExtentInInches();
    double distance = currentPosition - m_extentInInches;

    if (Math.abs(m_extentInInches) < k_deadZone) {
      m_subsystem.setPower(0);
    } else {
      m_subsystem.setPower(distance * k_p);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
