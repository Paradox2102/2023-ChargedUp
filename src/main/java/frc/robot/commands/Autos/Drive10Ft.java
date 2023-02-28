package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.subsystems.DriveSubsystem;

public class Drive10Ft extends SequentialCommandGroup {
    public Drive10Ft(DriveSubsystem subsystem){
addCommands(new CreatePathCommand(subsystem, k_path, true, false, "Drive10Ft"));
    }

    private static final Waypoint[] k_path = {
        new Waypoint(0, 0, Math.toRadians(90)),
        new Waypoint(0, 10, Math.toRadians(90)),
    };

// 0,0,90
// 0,10,90
}
