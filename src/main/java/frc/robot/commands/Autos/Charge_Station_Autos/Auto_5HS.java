// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Charge_Station_Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PurePursuitData;
import frc.robot.commands.SetArmPositionExtent;
import frc.robot.commands.SetGamePieceCommand;
import frc.robot.commands.Autos.CreatePathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_5HS extends SequentialCommandGroup {
  /** Creates a new Auto_4LS. */
  private final double k_maxSpeed = 4.000000;
  private final double k_maxAccel = 2.000000;
  private final double k_maxDecl = 2.000000;
  private final double k_maxJerk = 50.000000;
  public Auto_5HS(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Place cone high
      new IntakeCommand(intakeSubsystem, Constants.k_intakePower, true),
      new SetGamePieceCommand(armSubsystem, true),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.HIGH, () -> true),
      new WaitCommand(1),
      new IntakeCommand(intakeSubsystem, Constants.k_outakePower, true),
      new WaitCommand(.5),
      new SetGamePieceCommand(armSubsystem, false),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_straightUpExtent, Constants.k_straightUpAngle, () -> false, Constants.k_straightUpWrist, 0, 0, true),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, ArmPosition.MID, () -> true),
      new IntakeCommand(intakeSubsystem, 0, true),

      // Go onto charge station
      new CreatePathCommand(driveSubsystem, k_path, true, true, "Path 1", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5, false),

      // new ConditionalCommand(new BackUp1NotMobility(driveSubsystem), new AutoBalanceCommand(driveSubsystem), () -> driveSubsystem.getRobotY() - 10 < 2),

      new AutoBalanceCommand(driveSubsystem)
    );
  }
/*
-2.35,5.857,90
-2.35,11.5,90
*/
private static final Waypoint[] k_path = {
    new Waypoint(-4.13, 5.857, Math.toRadians(90)),
    new Waypoint(-4.13, 11.75, Math.toRadians(90))
};
}
