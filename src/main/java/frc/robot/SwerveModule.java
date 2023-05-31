// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

/** Add your docs here. */
public class SwerveModule {
  // HARDWARE
  public final WPI_TalonFX angleMotor;
  private final WPI_TalonFX driveMotor;
  public final CANCoder orientationEncoder;

  // INFORMATION
  private final double defensiveAngleDeg;
  private double wheelOrientation = 0.0;
  public final SwerveModuleInfo info;
  

  public SwerveModule(SwerveModuleInfo info) {
    this.info = info;
    this.angleMotor = new WPI_TalonFX(info.TURN_ID);
    this.driveMotor = new WPI_TalonFX(info.DRIVE_ID);
    this.orientationEncoder = new CANCoder(info.ENCODER_ID);
    this.defensiveAngleDeg = VectorR.fromCartesian(info.X, info.Y).getAngle();
    angleMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    orientationEncoder.setPosition(0);
    driveMotor.setSelectedSensorPosition(0);
  }

  //RESET METHODS
  public void resetDriveEncoder() {
    driveMotor.setSelectedSensorPosition(0);
  }

  /*
   * positive (+) = left turn CCW
   * negative (-) = right turn CW
   */
  public double getWheelOrientationDegrees() {
    return wheelOrientation- info.ABS_ENCODER_VALUE_WHEN_STRAIGHT;
  }

  // MODULE SPEEDS CALCULATIONS
  private VectorR desired = new VectorR();
  private boolean reversed = false;

  private void reverse() {
    reversed = !reversed;
  }

  private double desiredSpeed() {
    if (reversed)
      return desired.getTerminalMagnitude();
    else
      return desired.getMagnitude();
  }

  private double desiredAngle() {
    if (reversed)
      return desired.getTerminalAngle();
    else
      return desired.getAngle();
  }

  /*
   * UPDATE OR STOP METHODS MUST BE CALLED PERIODICALLY 
   * speed 0 min - 1 max, turns module drive wheel
   * angle degrees follows coordinate plane standards, sets module wheel to angle
   */
  public void update(double speed, double angleDegrees) {
    wheelOrientation = orientationEncoder.getAbsolutePosition();

    desired.setFromPolar(speed, angleDegrees);

    if (Math.abs(MathR.getDistanceToAngle(getWheelOrientationDegrees(), desiredAngle())) > 90d)
      reverse();

    double speed_power = MathR.limit(desiredSpeed(), -1, 1);
    double angle_power = MathR
        .limit(Constants.MODULE_ANGLE_KP * MathR.getDistanceToAngle(getWheelOrientationDegrees(), desiredAngle()), -1, 1);
   
  
    driveMotor.set(speed_power); 
    angleMotor.set(angle_power);

    
  }

  public void stop() {
    angleMotor.set(0);
    driveMotor.set(0);
  }

  public void stopDefensively() {
    update(0.0000001,  defensiveAngleDeg);
  }
}
