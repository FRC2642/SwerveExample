// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickOrientedDriveCommand;
import frc.robot.subsystems.DriveSubsystem;


public class RobotContainer {
  DriveSubsystem drive = new DriveSubsystem();
  
  XboxController control = new XboxController(Constants.DRIVE_CONTROL_PORT);
  
  public RobotContainer() {
    
    configureBindings();
  }

  
  private void configureBindings() {
   
  }

  public void teleopInit(){
    drive.setDefaultCommand(new JoystickOrientedDriveCommand(drive, control));
  }

 
  public Command getAutonomousCommand() {
    return null;
  }
}
