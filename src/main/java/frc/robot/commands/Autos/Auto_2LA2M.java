
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
import frc.robot.commands.DeliverGamePieceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmPositionExtent;
import frc.robot.commands.SetClawCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_2LA2M extends SequentialCommandGroup {
  /** Creates a new Auto_2LA2M. */
  public Auto_2LA2M(DriveSubsystem driveSubsystem, ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_groundPickupExtent, Constants.k_groundPickupAngle, () -> false, 0, 0, false),
      new IntakeCommand(intakeSubsystem, -.3, true),
      new CreatePathCommand(driveSubsystem, k_path1, true, false, "Path 1"),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_midConeNodeExtent, Constants.k_midConeNodeAngle, () -> true, 0, 0, false),
      new CreatePathCommand(driveSubsystem, k_path2, true, true, "Path 2"),
      new IntakeCommand(intakeSubsystem, .3, true),
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.OPEN)
    );
  }

    private static final Waypoint[] k_path1 = {
      new Waypoint(1.163,  5.857, Math.toRadians(90), 1.335, 5.575),
      new Waypoint(2.3,  21.5, Math.toRadians(99.0))
  };

  private static final Waypoint[] k_path2 = {
    new Waypoint(2.3,  21.5, Math.toRadians(279)),
    new Waypoint(1.163,  5.857, Math.toRadians(-90), 1.335, 5.575)
  };
}



