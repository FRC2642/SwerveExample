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

  public static final int DRIVE_CONTROL_PORT = 1;

  public static final int FRONT_RIGHT_TURN_MOTOR_ID = 7;
  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 8;
  public static final int FRONT_RIGHT_ABS_ENCODER_ID = 14;

  public static final int FRONT_LEFT_TURN_MOTOR_ID = 1;
  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 2;
  public static final int FRONT_LEFT_ABS_ENCODER_ID = 11;

  public static final int BACK_RIGHT_TURN_MOTOR_ID = 5;
  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 6;
  public static final int BACK_RIGHT_ABS_ENCODER_ID = 13;

  public static final int BACK_LEFT_TURN_MOTOR_ID = 9;
  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 4;
  public static final int BACK_LEFT_ABS_ENCODER_ID = 12;
  
}
