// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WristPIDSS;

public class WristMovementC extends Command {

  public final WristPIDSS m_Wrist;

  public WristMovementC(WristPIDSS subsystem) {
    m_Wrist = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    WristPIDSS.Wrist.setNeutralMode(NeutralMode.Brake);
    
    if (RobotContainer.aButtonOp.getAsBoolean() == true) {
      RobotContainer.rc_WristPIDSS.Spin(-0.5);
    }
    else if (RobotContainer.bButtonOp.getAsBoolean() == true) {
      RobotContainer.rc_WristPIDSS.Spin(0.5);
    }
    else if (RobotContainer.aButtonOp.getAsBoolean() == false) {
      RobotContainer.rc_WristPIDSS.Spin(0);
    }
    else if (RobotContainer.bButtonOp.getAsBoolean() == false) {
      RobotContainer.rc_WristPIDSS.Spin(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.rc_WristPIDSS.Stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
