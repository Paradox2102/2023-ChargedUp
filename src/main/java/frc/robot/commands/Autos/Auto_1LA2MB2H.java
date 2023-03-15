// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PurePursuitData;
import frc.robot.commands.SetArmPositionExtent;
import frc.robot.commands.SetClawCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_1LA2MB2H extends SequentialCommandGroup {
  /** Creates a new Auto_1LA2MB2H. */
  public Auto_1LA2MB2H(DriveSubsystem driveSubsystem, ReachSubsystem reachSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final double k_maxSpeed = 7.000000;
    final double k_maxAccel = 5.000000;
    final double k_maxDecl = 5.000000;
    final double k_maxJerk = 50.000000;

    addCommands(
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_groundPickupExtent, Constants.k_groundPickupAngle, () -> false, 0, 0, false),
      new IntakeCommand(intakeSubsystem, -.3, true),
      new CreatePathCommand(driveSubsystem, k_path1, true, false, "Path 1", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk)),
      new SetArmPositionExtent(reachSubsystem, armSubsystem, Constants.k_midConeNodeExtent, Constants.k_midConeNodeAngle, () -> true, 0, 0, false),
      new CreatePathCommand(driveSubsystem, k_path2, true, true, "Path 2"),
      new IntakeCommand(intakeSubsystem, .3, true),
      new SetClawCommand(intakeSubsystem, IntakeSubsystem.ClawPosition.OPEN)
    );
  }
  /*
  3.323,5.525,90
  1.952,22.5,90
  */
  private static final Waypoint[] k_path1 = {
      new Waypoint(3.323, 5.525, Math.toRadians(90)),
      new Waypoint(1.952, 22.5, Math.toRadians(90))
  };
  /*
  1.952,22.5,270
  1.412,5.4,270
  */
  private static final Waypoint[] k_path2 = {
      new Waypoint(1.952, 22.5, Math.toRadians(270)),
      new Waypoint(1.412, 5.4, Math.toRadians(270))
  };
  /*
  1.412,5.4,84.532,8.721,5.779
  -1.828,22.431,120
  */
  private static final Waypoint[] k_path3 = {
      new Waypoint(1.412, 5.4, Math.toRadians(84.532), 8.721, 5.779),
      new Waypoint(-1.828, 22.431, Math.toRadians(120))
  };
}
