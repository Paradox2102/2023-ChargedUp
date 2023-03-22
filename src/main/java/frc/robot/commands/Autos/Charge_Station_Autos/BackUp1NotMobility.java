// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Charge_Station_Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.commands.PurePursuitData;
import frc.robot.commands.Autos.CreatePathCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackUp1NotMobility extends SequentialCommandGroup {
  /** Creates a new BackUp1NotMobility. */
  private final double k_maxSpeed = 4.000000;
  private final double k_maxAccel = 2.000000;
  private final double k_maxDecl = 2.000000;
  private final double k_maxJerk = 50.000000;
  public BackUp1NotMobility(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CreatePathCommand(driveSubsystem, k_path1, false, true, "Path 1", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5, false),
      new WaitCommand(1),
      new CreatePathCommand(driveSubsystem, k_path2, false, false, "Path 2", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5, false)
    );
  }
  /*
  -2.35,10.5,-90
  -2.35,8,-90
  */
  private static final Waypoint[] k_path1 = {
    new Waypoint(-4.13, 10.5, Math.toRadians(-90)),
    new Waypoint(-4.13, 8, Math.toRadians(-90))
  };
  /*
  -2.35,8,90
  -2.35,13,90
  */
  private static final Waypoint[] k_path2 = {
      new Waypoint(-4.13, 8, Math.toRadians(90)),
      new Waypoint(-4.13, 13, Math.toRadians(90))
  };
}
