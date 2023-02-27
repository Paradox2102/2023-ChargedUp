// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.OptionalDouble;
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

  private static final double k_p = 0.1;
  private static final double k_deadZone = 1;

  private boolean m_constructor1;
  private boolean m_isFinished = false;

  /** Creates a new SetArmExtent. */
  public SetArmPositionExtent(ReachSubsystem reachSubsystem, ArmSubsystem armSystem, double extentInInches, double armAngleInDegrees, BooleanSupplier throttle) {
    m_reachSubsystem = reachSubsystem;
    m_armSubsystem = armSystem;
    m_armAngleInDegrees = armAngleInDegrees;
    m_throttle = throttle;
    m_extentInInches = extentInInches;
    if (m_extentInInches < 0) {
      m_extentInInches = 0;
    }
    else if (m_extentInInches > Constants.k_maxArmLength) {
      m_extentInInches = Constants.k_maxArmLength; 
    }

    m_constructor1 = true;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_reachSubsystem, m_armSubsystem);
  }

  public SetArmPositionExtent(ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, BooleanSupplier throttle) {
    m_armSubsystem = armSubsystem;
    m_reachSubsystem = reachSubsystem;
    m_throttle = throttle;
    m_constructor1 = false;
    addRequirements(m_armSubsystem, m_reachSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("SetArmPositionExtent", 1, "initialize");
    if (!m_constructor1) {
      OptionalDouble armAngleInDegrees = m_armSubsystem.computeTargetAngleInDegrees();
      OptionalDouble extentInInches = m_armSubsystem.computeTargetDistance();
      if (armAngleInDegrees.isPresent() && extentInInches.isPresent()){
        m_armAngleInDegrees = armAngleInDegrees.getAsDouble();
        m_extentInInches = extentInInches.getAsDouble();
        SmartDashboard.putNumber("Target Extent In Inches", m_extentInInches);
        SmartDashboard.putNumber("Target Angle In Degrees", m_armAngleInDegrees);
        m_armSubsystem.moveToAngle(m_throttle.getAsBoolean() ? m_armAngleInDegrees : -m_armAngleInDegrees);
      } else {
        m_isFinished = true;
      }
    } else {
      m_armSubsystem.moveToAngle(m_throttle.getAsBoolean() ? m_armAngleInDegrees : -m_armAngleInDegrees);

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = m_reachSubsystem.getExtentInInches();
    double distance = m_extentInInches - currentPosition;

    if (Math.abs(distance) < k_deadZone) {
      m_reachSubsystem.setPower(0);
    } else {
      m_reachSubsystem.setPower(distance * k_p);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.log("SetArmPositionExtent", 1, "end");
    m_armSubsystem.enable(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
