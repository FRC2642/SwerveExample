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
  private double abs_encoder_value_max_value;
  public final double MODULE_TANGENT_DEG;

  public SwerveModule(int drive_motor_CAN_ID, int angle_motor_CAN_ID, int encoder_CAN_ID, double abs_encoder_max_value,
  double abs_encoder_value_when_wheel_straight, double x, double y) {
      
      this.angleMotor = new WPI_TalonFX(angle_motor_CAN_ID);
      this.driveMotor = new WPI_TalonFX(drive_motor_CAN_ID);
      this.orientationEncoder = new CANCoder(encoder_CAN_ID);
      this.defensiveAngleDeg = VectorR.fromCartesian(x, y).getAngle();
      this.abs_encoder_value_max_value = abs_encoder_max_value;
      angleMotor.setNeutralMode(NeutralMode.Brake);
      driveMotor.setNeutralMode(NeutralMode.Brake);
      orientationEncoder.setPosition(0);
      driveMotor.setSelectedSensorPosition(0);
      MODULE_TANGENT_DEG = VectorR.fromCartesian(x, y).getAngle() + 90;
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
    return wheelOrientation- abs_encoder_value_max_value;
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
