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
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;

public class SetArmPositionExtent extends CommandBase {
  private ReachSubsystem m_reachSubsystem;
  private ArmSubsystem m_armSubsystem;
  private WristSubsystem m_wristSubsystem;
  private double m_armAngleInDegrees;
  // private double m_wristAngleInDegrees;
  private BooleanSupplier m_throttle;
  private double m_extentInInches;


  private boolean m_manualTarget;
  private boolean m_isFinished = false;
  private double m_changeBatteryAngle = 0;
  private double m_changeMotorAngle = 0;
  private boolean m_armStraightUp = false;
  private double m_wristAngleInDegrees;
  private ArmPosition m_armPos = null;
  private boolean m_usePresets = false;

  /** Creates a new SetArmExtent. */
  public SetArmPositionExtent(ReachSubsystem reachSubsystem, ArmSubsystem armSystem, WristSubsystem wristSubsystem, double extentInInches, double armAngleInDegrees, BooleanSupplier throttle, double wristAngleInDegrees, double changeBatteryAngle, double changeMotorAngle, boolean armStraightUp) {
    m_reachSubsystem = reachSubsystem;
    m_armSubsystem = armSystem;
    m_wristSubsystem = wristSubsystem;
    m_armAngleInDegrees = armAngleInDegrees;
    m_throttle = throttle;
    m_extentInInches = extentInInches;
    m_changeMotorAngle = changeMotorAngle;
    m_changeBatteryAngle = changeBatteryAngle;
    m_armStraightUp = armStraightUp;
    m_wristAngleInDegrees = wristAngleInDegrees;
    if (m_extentInInches < 0) {
      m_extentInInches = 0;
    }
    else if (m_extentInInches > Constants.k_maxArmLength) {
      m_extentInInches = Constants.k_maxArmLength; 
    }

    m_manualTarget = true;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_reachSubsystem, m_armSubsystem, m_wristSubsystem);
  }

  public SetArmPositionExtent(ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, ArmPosition armPos, BooleanSupplier throttle) {
    m_reachSubsystem = reachSubsystem;
    m_armSubsystem = armSubsystem;
    m_wristSubsystem = wristSubsystem;
    m_armPos = armPos;
    m_throttle = throttle;

    m_manualTarget = true;
    m_usePresets = true;
    addRequirements(m_reachSubsystem, m_armSubsystem, m_wristSubsystem);
  }

  // For TurnToTarget
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
    } else if (m_usePresets) {
      double extent = 0;
      double wristAngle = 0;
      double armAngle = 0;
      if (m_armSubsystem.isGamePieceCube()) {
        switch (m_armPos) {
          case HIGH:
            extent = Constants.k_topNodeExtentCUBE;
            wristAngle = Constants.k_topNodeWristCUBE;
            armAngle = Constants.k_topNodeAngleCUBE;
            break;
          case MID:
            extent = Constants.k_midNodeExtentCUBE;
            wristAngle = Constants.k_midNodeWristCUBE;
            armAngle = Constants.k_midNodeAngleCUBE;
            break;
          case LOW:
            extent = Constants.k_groundPickupExtentCUBE;
            wristAngle = Constants.k_groundPickupWristCUBE;
            armAngle = Constants.k_groundPickupAngleCUBE;
            break;
          case SUBSTATION:
            extent = Constants.k_humanPlayerStationExtentCUBE;
            wristAngle = Constants.k_humanPlayerStationWristCUBE;
            armAngle = Constants.k_humanPlayerStationAngleCUBE;
            break;
        } 
        Logger.log("SetArmPositionExtent", 1, String.format("is cube=true, arm angle=%f, extent=%f, wrist=%f, armPos=%s", armAngle, extent, wristAngle, m_armPos));
      } else {
        switch (m_armPos) {
          case HIGH:
            extent = Constants.k_topExtentCONE;
            wristAngle = Constants.k_topNodeWristCONE;
            armAngle = Constants.k_topNodeAngleCONE;
            break;
          case MID:
            extent = Constants.k_midNodeExtentCONE;
            wristAngle = Constants.k_midNodeWristCONE;
            armAngle = Constants.k_midNodeAngleCONE;
            break;
          case LOW:
            extent = Constants.k_groundPickupExtentCONE;
            wristAngle = Constants.k_groundPickupWristCONE;
            armAngle = Constants.k_groundPickupAngleCONE;
            break;
          case SUBSTATION:
            extent = Constants.k_humanPlayerStationExtentCONE;
            wristAngle = Constants.k_humanPlayerStationWristCONE;
            armAngle = Constants.k_humanPlayerStationAngleCONE;
            break;
        }
      }
        m_reachSubsystem.isRunP(false);
        m_armSubsystem.moveToAngle(m_throttle.getAsBoolean() ? armAngle : -armAngle);
        m_wristSubsystem.setPosition(m_throttle.getAsBoolean() ? wristAngle : -wristAngle - 1.5);
        m_extentInInches = extent;
    } else {
      if (m_armStraightUp) {
        m_armSubsystem.moveToAngle(m_throttle.getAsBoolean() ? m_armAngleInDegrees + m_changeMotorAngle: -m_armAngleInDegrees + m_changeBatteryAngle);
        m_reachSubsystem.isRunP(true);
        if (Constants.k_isCompetition) {
          m_wristSubsystem.setPosition(m_throttle.getAsBoolean() ? m_wristAngleInDegrees : -m_wristAngleInDegrees);
        }
        m_isFinished = true;
      } else {
        m_reachSubsystem.isRunP(false);
        m_armSubsystem.moveToAngle(m_throttle.getAsBoolean() ? m_armAngleInDegrees + m_changeMotorAngle: -m_armAngleInDegrees + m_changeBatteryAngle);
        if (Constants.k_isCompetition) {
          m_wristSubsystem.setPosition(m_throttle.getAsBoolean() ? m_wristAngleInDegrees : -m_wristAngleInDegrees);
        }
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
