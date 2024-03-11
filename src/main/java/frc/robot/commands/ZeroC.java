// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

public class ZeroC extends Command {

  private boolean zeroElevator;

  public ZeroC(PIDSS subsystem) {
    addRequirements(subsystem);
  }
  
  public ZeroC() {
  }

  @Override
  public void initialize() {
    zeroElevator = false;
    if (RobotContainer.startButtonDr.getAsBoolean() == true) {
        RobotContainer.rc_driveSS.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
        RobotContainer.rc_driveSS.m_gyro.setYaw(0);
      }
  }

  @Override
  public void execute() {  
    if(RobotContainer.startButtonOp.getAsBoolean() == true) {
      RobotContainer.rc_pidSS.Elevator.set(-.4);
      zeroElevator = true;
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    if(zeroElevator == true){
      RobotContainer.rc_pidSS.Elevator.getEncoder().setPosition(0);
      zeroElevator = false;
    }  
    if (RobotContainer.startButtonDr.getAsBoolean() == true) {
        RobotContainer.rc_driveSS.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
        RobotContainer.rc_driveSS.m_gyro.setYaw(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
