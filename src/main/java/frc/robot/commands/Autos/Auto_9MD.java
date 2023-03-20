// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
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
public class Auto_9MD extends SequentialCommandGroup {
  /** Creates a new Auto_9MD. */
  public Auto_9MD(ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CONE),
      new WaitCommand(1),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, 8, Constants.k_midNodeAngleCUBE, () -> true, Constants.k_midConeNodeWristCUBE, 0, -8, false),
      new WaitCommand(1),
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE),
      new WaitCommand(1),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, wristSubsystem, Constants.k_straightUpExtent, Constants.k_straightUpAngle, () -> true, Constants.k_straightUpWrist, 0, 0, true),
      new WaitCommand(1),
      new CreatePathCommand(driveSubsystem, k_path, true, false, "Path 1")
    );
  }
  /*
  -11.41,5.649,90,3.618,5.896
  -10.5,22.306,436.464
  */
  private static final Waypoint[] k_path = {
      new Waypoint(-11.41, 5.649, Math.toRadians(90), 3.618, 5.896),
      new Waypoint(-10.5, 22.306, Math.toRadians(436.464))
  };
}
