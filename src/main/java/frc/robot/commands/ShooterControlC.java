// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSS;

public class ShooterControlC extends Command {

  public ShooterControlC(ShooterSS subsystem, double speed) {
    subsystem = RobotContainer.rc_ShooterSS;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    if(RobotContainer.RightBumperOp.getAsBoolean() == true) {
      RobotContainer.rc_ShooterSS.AmpShoot();
    }
    else{
      if(RobotContainer.LeftBumperOp.getAsBoolean() == true) {
        RobotContainer.rc_ShooterSS.SpeakerShoot();
      }
      else{
        if(RobotContainer.yButtonDr.getAsBoolean() == true) {
          RobotContainer.rc_ShooterSS.load();
        }
        else{
          if(RobotContainer.xButtonDr.getAsBoolean() == true){
            RobotContainer.rc_ShooterSS.load();
          }
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.rc_ShooterSS.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
