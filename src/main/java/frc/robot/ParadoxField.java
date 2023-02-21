// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ParadoxField {
    private static final double k_feetPerMeter = 3.28084;

    // Returns the same angle constrained such that -180 <= angle <= 180
    public static double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle <= -180) {
            angle += 360;
        } else if (angle >= 180) {
            angle -= 360;
        }
        return angle;
    }

    // Returns the FRC angle from the Paradox angle
    public static Rotation2d rotation2dFromParadox(Rotation2d angle) {
        return Rotation2d.fromDegrees(normalizeAngle(angle.getDegrees() - 90));
    }

    // Returns the FRC angle from the Paradox angle in degrees
    public static Rotation2d rotation2dFromParadoxAngle(double angleInDegrees)
    {
        return Rotation2d.fromDegrees(normalizeAngle(angleInDegrees - 90));
    }

    // Returns the Paradox angle from FRC angle
    public static Rotation2d rotation2dFromFRC(Rotation2d angle) {
        return Rotation2d.fromDegrees(normalizeAngle(angle.getDegrees() + 90));
    }

    // Returns the FRC Pose2d (in meters) from the paradox Pose2d (in feet)
    public static Pose2d pose2dFromParadox(Pose2d pos)
    {
        return pose2dFromParadox(pos.getX(), pos.getY(), pos.getRotation().getDegrees());
    }

    // Returns the FRC Pose2d (in meters) from the paradox x, y (in feet), and angle (in degrees)
    public static Pose2d pose2dFromParadox(double x, double y, double angleInDegrees)
    {
        return new Pose2d(y / k_feetPerMeter, -x / k_feetPerMeter, rotation2dFromParadoxAngle(angleInDegrees));
    }

    // Returns the Paradox Pose2d (in feet) from the FRC Pose2d (in meters)
    public static Pose2d pose2dFromFRC(Pose2d pos)
    {
        return new Pose2d(-pos.getY() * k_feetPerMeter, pos.getX() * k_feetPerMeter, rotation2dFromFRC(pos.getRotation()));
    }

    // Returns the Paradox distance (in feet) from the FRC distance (in meters)
    public static double distanceFromFRC(double dist)
    {
        return dist * k_feetPerMeter;
    }

    // Returns the FRC distance (in meters) from the FRC distance (in feet)
    public static double distanceFromParadox(double dist)
    {
        return dist / k_feetPerMeter;
    }
}
