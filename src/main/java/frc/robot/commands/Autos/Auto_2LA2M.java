
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
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_2LA2M extends SequentialCommandGroup {
  /** Creates a new Auto_2LA2M. */
  public Auto_2LA2M(DriveSubsystem driveSubsystem, ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_groundPickupExtentCUBE, Constants.k_groundPickupAngleCUBE, () -> false, Constants.k_groundPickupWristCUBE, 0, 0, false),
      new IntakeCommand(intakeSubsystem, -.3, true),
      new CreatePathCommand(driveSubsystem, k_path1, true, false, "Path 1"),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_midNodeExtentCUBE, Constants.k_midNodeAngleCUBE, () -> true, Constants.k_midConeNodeWristCUBE, 0, 0, false),
      new CreatePathCommand(driveSubsystem, k_path2, false, true, "Path 2"),
      new IntakeCommand(intakeSubsystem, .3, true),
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.OPEN),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_straightUpExtent, Constants.k_straightUpAngle, () -> true, Constants.k_straightUpWrist, 0, 0, true),
      new IntakeCommand(intakeSubsystem, .3, true)
    );
  }
    /*
    1.37, 5.857, 90
    2.3, 21.5, 99
    */
    private static final Waypoint[] k_path1 = {
      new Waypoint(1.37,  5.857, Math.toRadians(90), 1.335, 5.575),
      new Waypoint(2.3,  21.5, Math.toRadians(99.0))
  };
  /*
  2.3, 21.5, 279
  1.37, 5.857, -90
  */
  private static final Waypoint[] k_path2 = {
    new Waypoint(2.3,  21.5, Math.toRadians(279)),
    new Waypoint(1.37,  5.857, Math.toRadians(-90), 1.335, 5.575)
  };
}



