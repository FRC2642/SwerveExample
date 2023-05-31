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
  public static final double MODULE_ANGLE_KP = 0.00524;

  public static final int DRIVE_CONTROL_PORT = 0;

  // Swerve
  public static final SwerveModuleInfo FRONT_RIGHT = new SwerveModuleInfo(8, 7, 14, 360, 64.599, 1, -1);
  public static final SwerveModuleInfo FRONT_LEFT = new SwerveModuleInfo(2, 1, 11, 360, 67.5, 1, 1);
  public static final SwerveModuleInfo BACK_RIGHT = new SwerveModuleInfo(6, 5, 13, 360, 288.28, -1, -1);
  public static final SwerveModuleInfo BACK_LEFT = new SwerveModuleInfo(4, 9, 12, 360, 1.2304, -1, 1);

  
}
