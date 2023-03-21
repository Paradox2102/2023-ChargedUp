// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Auto_4LSCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.commands.PurePursuitData;
import frc.robot.commands.Autos.CreatePathCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackUp2Mobility extends SequentialCommandGroup {
  /** Creates a new BackUp2. */
  private final double k_maxSpeed = 6.000000;
  private final double k_maxAccel = 4.000000;
  private final double k_maxDecl = 4.000000;
  private final double k_maxJerk = 50.000000;
  public BackUp2Mobility(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Back up
      new CreatePathCommand(driveSubsystem, k_path1, false, false, "Path 1", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5, false),
      new WaitCommand(1),

      // Get on charge station
      new CreatePathCommand(driveSubsystem, k_path2, false, true , "Path 2", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5, false)
    );
  }
  /*
  -2.35,14.5,90
  -2.35,19,90
  */
  private static final Waypoint[] k_path1 = {
      new Waypoint(-2.35, 14.5, Math.toRadians(90)),
      new Waypoint(-2.35, 19, Math.toRadians(90))
  };
  /*
  -2.35,19,-90
  -2.35,12.5,-90
  */
  private static final Waypoint[] k_path2 = {
      new Waypoint(-2.35, 19, Math.toRadians(-90)),
      new Waypoint(-2.35, 12.5, Math.toRadians(-90))
  };
}
