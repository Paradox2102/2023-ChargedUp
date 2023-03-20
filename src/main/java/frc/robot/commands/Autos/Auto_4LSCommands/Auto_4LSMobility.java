// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Auto_4LSCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.PurePursuitData;
import frc.robot.commands.Autos.CreatePathCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReachSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_4LSMobility extends SequentialCommandGroup {
  private final double k_maxSpeed = 6.000000;
  private final double k_maxAccel = 4.000000;
  private final double k_maxDecl = 4.000000;
  private final double k_maxJerk = 50.000000;
  /** Creates a new Auto_4LSMobility. */
  public Auto_4LSMobility(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ReachSubsystem reachSubsystem, WristSubsystem wristSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CreatePathCommand(driveSubsystem, k_path1, true, false, "Path 1", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5, true),

      // PROBLEM: This test will be performed at command creation, not while the command is running.  You want to use something like:
      //   new ConditionalCommand(new BackUp1(driveSubsystem), new WaitCommand(1), () -> driveSubsystem.getRobotY() - 10 < 2)
      // -Gavin
      // Is the robot stuck?
      driveSubsystem.getRobotY() - 10 < 2 ? new BackUp1(driveSubsystem) : new WaitCommand(1),
    
      new CreatePathCommand(driveSubsystem, k_path2, false, true, "Path 2", new PurePursuitData(k_maxSpeed, k_maxAccel, k_maxDecl, k_maxJerk), .5, true),

      // PROBLEM: Same as above.  -Gavin

      // Is the robot stuck?
      driveSubsystem.getRobotY() - 14 < 2 ? new BackUp2(driveSubsystem) : new WaitCommand(1),

      new AutoBalanceCommand(driveSubsystem)

      
    );
  }

  /*
  -2.35,5.857,90
  -2.35,20,90
  */
  private static final Waypoint[] k_path1 = {
      new Waypoint(-2.35, 5.857, Math.toRadians(90)),
      new Waypoint(-2.35, 20, Math.toRadians(90))
  };
  /*
  -2.35,20,-90
  -2.35,12.5,-90
  */
  private static final Waypoint[] k_path2 = {
      new Waypoint(-2.35, 20, Math.toRadians(-90)),
      new Waypoint(-2.35, 12.5, Math.toRadians(-90))
  };
}
