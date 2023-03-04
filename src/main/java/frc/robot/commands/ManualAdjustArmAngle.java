// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.ApriltagsCamera.Logger;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualAdjustArmAngle extends InstantCommand {
  ArmSubsystem m_subsystem;
  BooleanSupplier m_switchSides;
  double m_degrees;
  public ManualAdjustArmAngle(ArmSubsystem armSubsystem, BooleanSupplier switchSides, double degrees) {
    m_subsystem = armSubsystem;
    m_switchSides = switchSides;
    m_degrees = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.log("ManualAdjustArmAngle", 1, "initialize");
    m_subsystem.addToTargetAngle(m_degrees, m_switchSides.getAsBoolean());
  }
}
