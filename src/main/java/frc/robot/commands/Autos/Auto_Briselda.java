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
public class Auto_Briselda extends SequentialCommandGroup {
  /** Creates a new Auto_Briselda. */
  public Auto_Briselda(ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CONE),
    new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_topNodeExtentCUBE, Constants.k_topNodeAngleCUBE, () -> true, Constants.k_topConeNodeWristCUBE, 0, 0, false),
    new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE),
    new IntakeCommand(intakeSubsystem, .3, true),
    new CreatePathCommand(driveSubsystem, k_path1, true, false, "Path 1"),
    new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_midNodeExtentCUBE, Constants.k_midNodeAngleCUBE, () -> false, Constants.k_midConeNodeWristCUBE, 0, 0, false),
    new CreatePathCommand(driveSubsystem, k_path2, false, true, "Path 2"),
    new IntakeCommand(intakeSubsystem, -.3, true),
    new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.OPEN));
  }



  /*
   * -12,6,90
   * -10.4,22.5,90
   */
  private static final Waypoint[] k_path1 = {
      new Waypoint(-12, 6, Math.toRadians(90)),
      new Waypoint(-10.4, 22.5, Math.toRadians(90))
  };

/*
-10.4,22.5,-90
-12,6,-90
*/
private static final Waypoint[] k_path2 = {
    new Waypoint(-10.4, 22.5, Math.toRadians(-90)),
    new Waypoint(-12, 6, Math.toRadians(-90))
};
}
