// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int k_rightDrive = 2;
  public static final int k_rightFollower = 12;
  public static final int k_leftDrive = 5;
  public static final int k_leftFollower = 0;

  public static final double k_feetPerTick = 12.58 / 188529;
  public static final double k_rampTimeSeconds = .1;

  public static final boolean k_xboxController = true;
  
  public static class OperatorConstants {}
}
