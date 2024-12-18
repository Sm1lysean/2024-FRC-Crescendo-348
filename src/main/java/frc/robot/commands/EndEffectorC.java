// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class EndEffectorC extends InstantCommand {
  /** Creates a new EndEffectorC. */
  public EndEffectorC(PneumaticsSS subsystem) {
    subsystem = RobotContainer.rc_pneumaticsSS;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.rc_pneumaticsSS.EndEffect();
  }
}
