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
public class Auto_9LD9M extends SequentialCommandGroup {
  /** Creates a new Auto_8LD8M. */
  public Auto_9LD9M(DriveSubsystem driveSubsystem, ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_groundPickupExtent, Constants.k_groundPickupAngle, () -> true, 0, 0, false),
      new IntakeCommand(intakeSubsystem, -.3, true),
      new CreatePathCommand(driveSubsystem, k_path1, true, false, "Path 1"),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_midConeNodeExtent, Constants.k_midConeNodeAngle, () -> false, 0, 0, false),
      new CreatePathCommand(driveSubsystem, k_path2, true, false, "Path 2"),
      new IntakeCommand(intakeSubsystem, .5, true) 
    );
  }

  /*
  -11.41,5.649,90,3.618,5.896
  -10.5,22.306,436.464
  */
  private static final Waypoint[] k_path1 = {
    new Waypoint(-11.41, 5.649, Math.toRadians(90), 3.618, 5.896),
    new Waypoint(-10.5, 22.306, Math.toRadians(436.464))
};

  /*
  -10.5,22.306,256.464,6.937,5.57
  -11.41,5.649,270
  */
  private static final Waypoint[] k_path2 = {
      new Waypoint(-10.5, 22.306, Math.toRadians(256.464), 6.937, 5.57),
      new Waypoint(-11.41, 5.649, Math.toRadians(270))
  };
}


