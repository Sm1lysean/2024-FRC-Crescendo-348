// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSS extends SubsystemBase {
  /** Creates a new PneumaticsSS. */

  public final DoubleSolenoid endEffectorSol;
  public final DoubleSolenoid climberSol;


  public PneumaticsSS() {
    endEffectorSol = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.forwardendSol, Constants.reverseendSol);
    climberSol = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.forwardclimbSol, Constants.reverseclimbSol);
    endEffectorSol.set(Value.kReverse);
    climberSol.set(Value.kReverse);
  }


  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    getEndEffector();
    getClimber();
    SmartDashboard.putData("Climber Sol Status", climberSol);
    SmartDashboard.putData("End Effector Status", endEffectorSol);
  }

  public void Climb() {
    climberSol.toggle();
  }

  public void EndEffect() {
    endEffectorSol.toggle();
  }

  public void getClimber() {
    climberSol.get();
  }

  public void getEndEffector(){
    endEffectorSol.get();
  }

  public void Reverse() {
    endEffectorSol.set(Value.kReverse);
    climberSol.set(Value.kReverse);
  }
}
