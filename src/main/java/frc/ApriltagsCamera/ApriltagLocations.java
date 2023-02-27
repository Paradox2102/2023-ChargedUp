// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.ApriltagsCamera;

import frc.robot.ParadoxField;

/** Add your docs here. */
public class ApriltagLocations {
    public static ApriltagLocation m_tags[] = { new ApriltagLocation(1, -9.63 * 12, 3.37 * 12, -90), 
                                                new ApriltagLocation(2, -4.13 * 12, 3.37 * 12, -90),
                                                new ApriltagLocation(3,  1.37 * 12, 3.37 * 12, -90),
                                                new ApriltagLocation(4,  9.00 * 12, 1.19 * 12, -90),
                                                new ApriltagLocation(5,  9.00 * 12, 53.08 * 12, 90),
                                                new ApriltagLocation(6,  1.37 * 12, 50.90 * 12, 90),
                                                new ApriltagLocation(7, -4.13 * 12, 50.90 * 12, 90),
                                                new ApriltagLocation(8, -9.63 * 12, 50.90 * 12, 90)
    };
    public static boolean m_blue = false;
    public static double m_fieldLength = 54.27 * 12;
    public static double m_fieldWidth = 26.29 * 12;

    public static void setColor(boolean blue)
    {
        if (blue != m_blue)
        {
            m_blue = blue;

            for (ApriltagLocation tag : m_tags)
            {
                tag.m_targetAngleDegrees = ParadoxField.normalizeAngle(tag.m_targetAngleDegrees + 180);
                tag.m_xInches = -tag.m_xInches;
                tag.m_yInches = m_fieldLength - tag.m_yInches;
            }
        }
    }

    public static ApriltagLocation findTag(int id)
    {
        for (ApriltagLocation tag : m_tags)
        {
            if (tag.m_tag == id)
            {
                return(tag);
            }
        }

        return(null);
    }
}
