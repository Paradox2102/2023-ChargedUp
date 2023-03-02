// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
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
public class Auto_8LD8M extends SequentialCommandGroup {
  /** Creates a new Auto_8LD8M. */
  public Auto_8LD8M(DriveSubsystem driveSubsystem, ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_groundPickupExtent, Constants.k_midConeNodeAngle, () -> false),
      new CreatePathCommand(driveSubsystem, k_path1, true, false, "Path 1"),
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE),
      new IntakeCommand(intakeSubsystem, -.25),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_groundPickupExtent, Constants.k_groundPickupAngle, () -> true),
      new CreatePathCommand(driveSubsystem, k_path2, true, false, "Path 2"),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_groundPickupExtent, Constants.k_groundPickupAngle, () -> false),
      new IntakeCommand(intakeSubsystem, .25) 
    );
  }

  private static final Waypoint[] k_path1 = {
      new Waypoint(-10.25, 5.774, Math.toRadians(90), 3.618, 5.833),
      new Waypoint(-10.5, 22.306, Math.toRadians(89))
  };

  private static final Waypoint[] k_path2 = {
    new Waypoint(-10.5, 22.306, Math.toRadians(271)),
    new Waypoint(-10.25, 5.774, Math.toRadians(270), 3.618, 5.833)
};
}


