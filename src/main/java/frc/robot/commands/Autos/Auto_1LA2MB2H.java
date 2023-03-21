// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PurePursuitData;
import frc.robot.commands.SetArmPositionExtent;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_1LA2MB2H extends SequentialCommandGroup {
  /** Creates a new Auto_1LA2MB2H. */

  private final double k_maxSpeed = 8.000000; // feet per second
  private final double k_maxAccel = 12.000000; // feet per second squared
  private final double k_maxDecl = 12.000000; // feet per second squared
  private final double k_maxJerk = 50.000000; // feet per second cubed

  public Auto_1LA2MB2H(DriveSubsystem driveSubsystem, ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Prepare to pick up game piece/knock first game piece in
      new SetGamePieceCommand(armSubsystem, true),
      
      // Go to game piece A
      new ParallelDeadlineGroup(
        new CreatePathCommand(driveSubsystem, k_path1, true, false, "Path 1", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5),
        new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.LOW, () -> false),
        new IntakeCommand(intakeSubsystem, Constants.k_intakePower, true)
        ),
      
      // Prepare to drop game piece A in mid cube node
      new ParallelDeadlineGroup(
        new CreatePathCommand(driveSubsystem, k_path2, false, true, "Path 2", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5),
        new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_straightUpExtent, Constants.k_straightUpAngle, () -> false, 15, 0, 0, false)
      ),

      // Drop game piece A
      new IntakeCommand(intakeSubsystem, Constants.k_outakePower, true),
      new WaitCommand(.15),


      // Go to game piece B
      new ParallelDeadlineGroup(
        new CreatePathCommand(driveSubsystem, k_path3, false, false, "Path 3", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5),
        new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.LOW, () -> false),
        new IntakeCommand(intakeSubsystem, Constants.k_intakePower, true)
      ),

      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.RESET, () -> true),

      // Prepare to drop game piece B in high cube node
      new ParallelDeadlineGroup(
        new CreatePathCommand(driveSubsystem, k_path4, false, true, "Path 4", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5),
        new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.HIGH, () -> true)
      ),

      // Drop Game piece B
      new IntakeCommand(intakeSubsystem, Constants.k_outakePower, true),
      new WaitCommand(.15),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.RESET, () -> true)
    );
  }
  /*
  3.15,5.857,90
  2.3,21.5,90
  */
  private static final Waypoint[] k_path1 = {
      new Waypoint(3.15, 5.857, Math.toRadians(90)),
      new Waypoint(2.3, 21.5, Math.toRadians(100))
  };
  /*
  2.3,21.5,-93,5.408,5.224
  1.37,5.857,-90
  */
  private static final Waypoint[] k_path2 = {
      new Waypoint(2.3, 21.5, Math.toRadians(-93), 5.408, 5.224),
      new Waypoint(  1.37, 5.857, Math.toRadians(-90))
  };
  /*
  1.37,5.857,81.985,8.581,6.022
  -1,22.348,134.721
  */
  private static final Waypoint[] k_path3 = {
      new Waypoint(1.37, 5.857, Math.toRadians(81.985), 8.581, 6.022),
      new Waypoint(-1, 22.348, Math.toRadians(139.721))
  };
  /*
  -1,22.348,-46.721,4.75,5.721
  1.37,5.4,-90,8.581,6.022
  */
  private static final Waypoint[] k_path4 = {
      new Waypoint(-1, 22.348, Math.toRadians(-41.721), 4.75, 5.721),
      new Waypoint(1.37, 6, Math.toRadians(-90), 8.581, 6.022)
  };
}
