// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverGamePieceCommand extends SequentialCommandGroup {
  /** Creates a new DeliverGamePieceCommand. */
  public DeliverGamePieceCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, BooleanSupplier switchSides) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnToTargetCommand(driveSubsystem, switchSides),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, /*Constants.k_isCompetition ? switchSides :*/ () -> !switchSides.getAsBoolean())
    );
  }
}
