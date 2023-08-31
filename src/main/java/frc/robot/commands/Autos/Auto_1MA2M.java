
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetArmPositionExtent;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;
//fruit name: Nectarine
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_1MA2M extends SequentialCommandGroup {
  /** Creates a new Auto_2LA2M. */
  public Auto_1MA2M(DriveSubsystem driveSubsystem, ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetGamePieceCommand(armSubsystem, false),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.MID, () -> true),
      new WaitCommand(1),
      new IntakeCommand(intakeSubsystem, Constants.k_outakePower, true),
      new WaitCommand(.5),
      new SetGamePieceCommand(armSubsystem, true),
      new WaitCommand(.5),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.RESET, () -> true),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.LOW, () -> false),
      new IntakeCommand(intakeSubsystem, Constants.k_intakePower, true),
      new CreatePathCommand(driveSubsystem, k_path1, true, true, "Path 1"),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.RESET, () -> true),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.MID, () -> true),
      new CreatePathCommand(driveSubsystem, k_path2, false, false, "Path 2"),
      new IntakeCommand(intakeSubsystem, Constants.k_outakePower, true)
    );
  }

    private static final Waypoint[] k_path1 = {
      new Waypoint(3.15,  5.857, Math.toRadians(90), 1.335, 5.575),
      new Waypoint(2.3,  21.5, Math.toRadians(99.0))
  };

  private static final Waypoint[] k_path2 = {
    new Waypoint(2.3,  21.5, Math.toRadians(279)),
    new Waypoint(1.37,  5.857, Math.toRadians(-90), 1.335, 5.575)
  };
}


