// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.utils.VectorR;

public class DriveSubsystem extends SubsystemBase {

  //HARDWARE
  private final ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>();
  private static AHRS gyro;

  //OTHER
  private boolean defensiveMode = true;
  private static double yawOffsetDegrees = 0;
  
  public DriveSubsystem() {
    SwerveModule frontLeft = new SwerveModule(Constants.FRONT_LEFT_DRIVE_MOTOR_ID, Constants.FRONT_LEFT_TURN_MOTOR_ID, Constants.FRONT_LEFT_ABS_ENCODER_ID, 360, 64.599, 1, -1);
    SwerveModule frontRight = new SwerveModule(Constants.FRONT_RIGHT_DRIVE_MOTOR_ID, Constants.FRONT_RIGHT_TURN_MOTOR_ID, Constants.FRONT_RIGHT_ABS_ENCODER_ID, 360, 67.5, 1, 1);
    SwerveModule backLeft = new SwerveModule(Constants.BACK_LEFT_DRIVE_MOTOR_ID, Constants.BACK_LEFT_TURN_MOTOR_ID, Constants.BACK_LEFT_ABS_ENCODER_ID, 360, 1.2304, -1, 1);
    SwerveModule backRight = new SwerveModule(Constants.BACK_RIGHT_DRIVE_MOTOR_ID, Constants.BACK_RIGHT_TURN_MOTOR_ID, Constants.BACK_RIGHT_ABS_ENCODER_ID, 360, 288.28, -1, -1);
  
    modules.add(frontLeft);
    modules.add(frontRight);
    modules.add(backLeft);
    modules.add(backRight);

    gyro = new AHRS();
    gyro.calibrate();
  }


  /*
   * SYSTEM STANDARD FOLLOWS COORDINATE PLANE STANDARD
   * positive (+) = forwards/left/left turn CCW
   * negative (-) = backwards/right/right turn CW
   * velocity magnitude (0-1) 1:fastest 0:stopped
   * turn (0-1)
   * NOTE: the speed of any wheel can reach a maximum of turn + |velocity|
   */
  public void move(VectorR directionalSpeed, double turnSpeed) {
    
    VectorR directionalPull = directionalSpeed.clone();
    directionalPull.rotate(-getYawDegrees());

    for (SwerveModule module : modules) {
      VectorR rotationalPull = VectorR.fromPolar(turnSpeed, module.MODULE_TANGENT_DEG);
      VectorR wheelPull = VectorR.addVectors(directionalPull, rotationalPull);

      module.update(wheelPull.getMagnitude(), wheelPull.getAngle());
    }
  }

  public void stop() {
    for (SwerveModule module : modules) {
      if (defensiveMode)
        module.stopDefensively();
      else
        module.stop();
    }
  }

  public void setDefensiveMode(boolean activated) {
    defensiveMode = activated;
  }

  public static double getYawDegrees() {
    return gyro.getYaw() + yawOffsetDegrees;
  }

  public static void resetGyro(double yawDegrees) {
    gyro.reset();
    gyro.calibrate();
    yawOffsetDegrees = yawDegrees;
  }
}
