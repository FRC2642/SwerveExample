// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class JoystickOrientedDriveCommand extends CommandBase {
  
  private double MAX_SPEED = 0.25;

  private final DriveSubsystem drive;
  private final XboxController control;
  private final VectorR leftJoystick = new VectorR();
  private final VectorR rightJoystick = new VectorR();

  final double TURN_KP = 0.017;
  
  
  public JoystickOrientedDriveCommand(DriveSubsystem drive, XboxController control) {
    this.drive = drive;
    this.control = control;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    DriveSubsystem.resetGyro(0.0);
  }

  @Override
  public void execute() {
    leftJoystick.setFromCartesian(control.getLeftX(), -control.getLeftY());
    leftJoystick.rotate(-90);
    rightJoystick.setFromCartesian(control.getRightX(), -control.getRightY());
    rightJoystick.rotate(90);

    if (leftJoystick.getMagnitude() < 0.1 && rightJoystick.getMagnitude() < 0.1) {
      drive.stop();
      return;
    }

    double angleToFace = rightJoystick.getAngle();
    double turnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(DriveSubsystem.getYawDegrees(), angleToFace), -1, 1);

    leftJoystick.mult(MAX_SPEED);
    drive.move(leftJoystick, turnPower * MAX_SPEED);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
