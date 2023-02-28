package frc.robot.commands.Autos.auto2LA2M;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.pathfinder.Pathfinder.Waypoint;
import frc.robot.commands.Autos.CreatePathCommand;
import frc.robot.subsystems.DriveSubsystem;

public class Path2 extends SequentialCommandGroup {
    public Path2(DriveSubsystem subsystem){
addCommands(new CreatePathCommand(subsystem, k_path, true, false, "Drive10Ft"));
    }

    private static final Waypoint[] k_path = {
        new Waypoint(1.911, 22.306, Math.toRadians(270)),
        new Waypoint(1.163, 5.857, Math.toRadians(270))
    };

// 0,0,90
// 0,10,90
} 