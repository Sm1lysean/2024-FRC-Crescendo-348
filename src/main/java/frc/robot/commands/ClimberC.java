// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;

public class ClimberC extends InstantCommand {
  private PneumaticsSS pneumaticsSS;
  /** Creates a new ClimberC. */
  public ClimberC(PneumaticsSS pneumaticsSS) {
    this.pneumaticsSS = pneumaticsSS;
    addRequirements(pneumaticsSS);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumaticsSS.Climb();
  }
}
