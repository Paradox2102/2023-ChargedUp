// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetArmPositionExtent;
import frc.robot.commands.SetClawCommand;
import frc.robot.commands.SetReachCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_4MS extends SequentialCommandGroup {
  /** Creates a new Auto_4LSTest. */
  public Auto_4MS(IntakeSubsystem intakeSubsystem, ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CONE),
      new WaitCommand(1),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_midNodeExtentCUBE, Constants.k_midNodeAngleCUBE, () -> true, Constants.k_midNodeWristCUBE, 0, 0, false),
      new WaitCommand(1),
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE),
      new WaitCommand(1),
      new SetReachCommand(reachSubsystem, 0),
      new Auto_4LS(driveSubsystem, armSubsystem, intakeSubsystem)
    );
  }
}
