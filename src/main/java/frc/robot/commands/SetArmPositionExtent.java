// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ReachSubsystem;

public class SetArmPositionExtent extends CommandBase {
  private ReachSubsystem m_reachSubsystem;
  private ArmSubsystem m_armSubsystem;
  private double m_armAngleInDegrees;
  private double m_wristAngleInDegrees;
  private double m_extentInInches;

  private static final double k_p = 0.1;
  private static final double k_deadZone = 1;

  /** Creates a new SetArmExtent. */
  public SetArmPositionExtent(ReachSubsystem reachSubsystem, ArmSubsystem armSystem, double extentInInches, double armAngleInDegrees, double wristAngleInDegrees) {
    m_reachSubsystem = reachSubsystem;
    m_armSubsystem = armSystem;
    m_armAngleInDegrees = armAngleInDegrees;
    m_wristAngleInDegrees = wristAngleInDegrees;
    m_extentInInches = extentInInches;
    if (m_extentInInches < 0) {
      m_extentInInches = 0;
    }
    else if (m_extentInInches > ReachSubsystem.k_maxArmLength) {
      m_extentInInches = ReachSubsystem.k_maxArmLength; 
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_reachSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.moveToAngle(m_armAngleInDegrees, m_wristAngleInDegrees); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = m_reachSubsystem.getExtentInInches();
    double distance = m_extentInInches - currentPosition;

    if (Math.abs(m_extentInInches) < k_deadZone) {
      m_reachSubsystem.setPower(0);
    } else {
      m_reachSubsystem.setPower(distance * k_p);
      // m_reachSubsystem.setPower(0.3 * Math.signum(distance)); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.enable(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
