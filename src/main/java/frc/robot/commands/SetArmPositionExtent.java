// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ApriltagsCamera.Logger;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ReachSubsystem;

public class SetArmPositionExtent extends CommandBase {
  private ReachSubsystem m_reachSubsystem;
  private ArmSubsystem m_armSubsystem;
  private double m_armAngleInDegrees;
  // private double m_wristAngleInDegrees;
  private BooleanSupplier m_throttle;
  private double m_extentInInches;


  private boolean m_manualTarget;
  private boolean m_isFinished = false;
  private double m_changeBatteryAngle;
  private double m_changeMotorAngle;
  private boolean m_armStraightUp;

  /** Creates a new SetArmExtent. */
  public SetArmPositionExtent(ReachSubsystem reachSubsystem, ArmSubsystem armSystem, double extentInInches, double armAngleInDegrees, BooleanSupplier throttle, double changeBatteryAngle, double changeMotorAngle, boolean armStraightUp) {
    m_reachSubsystem = reachSubsystem;
    m_armSubsystem = armSystem;
    m_armAngleInDegrees = armAngleInDegrees;
    m_throttle = throttle;
    m_extentInInches = extentInInches;
    m_changeMotorAngle = changeMotorAngle;
    m_changeBatteryAngle = changeBatteryAngle;
    m_armStraightUp = armStraightUp;
    if (m_extentInInches < 0) {
      m_extentInInches = 0;
    }
    else if (m_extentInInches > Constants.k_maxArmLength) {
      m_extentInInches = Constants.k_maxArmLength; 
    }

    m_manualTarget = true;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_reachSubsystem, m_armSubsystem);
  }

  public SetArmPositionExtent(ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, BooleanSupplier throttle) {
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_throttle = throttle;
    m_manualTarget = false;
    addRequirements(m_armSubsystem, m_reachSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;
    Logger.log("SetArmPositionExtent", 1, "initialize");
    if (!m_manualTarget) {
      // OptionalDouble armAngleInDegrees = null; //m_armSubsystem.computeTargetAngleInDegrees();
      // OptionalDouble extentInInches = m_armSubsystem.computeTargetDistance();
      double[] data = m_armSubsystem.computeTargetAngleInDegreesExtentInInches();
      // if (armAngleInDegrees.isPresent() && extentInInches.isPresent()) {
      if (data != null) {
        m_armAngleInDegrees = data[0]; //armAngleInDegrees.getAsDouble();
        m_extentInInches = data[1]; //extentInInches.getAsDouble();
        SmartDashboard.putNumber("Arm Extent In Inches", m_extentInInches);
        SmartDashboard.putNumber("Arm Angle In Degrees", m_armAngleInDegrees);
        m_armSubsystem.moveToAngle(m_throttle.getAsBoolean() ? m_armAngleInDegrees : -m_armAngleInDegrees);
        m_reachSubsystem.isRunP(true);
      } else {
        m_isFinished = true;
      }
    } else {
      if (m_armStraightUp) {
        m_armSubsystem.moveToAngle(m_throttle.getAsBoolean() ? m_armAngleInDegrees + m_changeMotorAngle: -m_armAngleInDegrees + m_changeBatteryAngle);
        m_reachSubsystem.isRunP(true);
        m_isFinished = true;
      } else {
        m_reachSubsystem.isRunP(false);
        m_armSubsystem.moveToAngle(m_throttle.getAsBoolean() ? m_armAngleInDegrees + m_changeMotorAngle: -m_armAngleInDegrees + m_changeBatteryAngle);
      }
    }
    m_reachSubsystem.setExtentInInches(m_extentInInches);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armSubsystem.isArmOnTarget() && !m_armStraightUp) {
      m_reachSubsystem.isRunP(true);
      m_isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SetArmPositionExtent", 1, "end");
    // m_armSubsystem.enable(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
