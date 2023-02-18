package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.subsystems.DriveSubsystem;

public class Drive10Ft extends SequentialCommandGroup {
    public Drive10Ft(DriveSubsystem subsystem){
addCommands(new CreatePathCommand(subsystem, k_path, true, true, "Drive10Ft"));
    }

    private static final Waypoint[] k_path = {
        new Waypoint(0, 0, Math.toRadians(90)),
        new Waypoint(-4, 8, Math.toRadians(90)),
        new Waypoint(4, 16, Math.toRadians(90)),
        new Waypoint(0, 24, Math.toRadians(90)),
    };

// 0,0,90
// -4,8,90
// 4,16,90
// 0,24,90
}
