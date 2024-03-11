// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSS extends SubsystemBase {
    public final static CANSparkMax shooterT = new CANSparkMax(Constants.shooterT, MotorType.kBrushless);
    public final static CANSparkMax shooterB = new CANSparkMax(Constants.shooterB, MotorType.kBrushless);

  public void AmpShoot() {
    shooterT.set(0.2);
    shooterB.set(0.2);
  }

  public void SpeakerShoot() {
    shooterT.set(1);
    shooterB.set(1);
  }

  public void load() {
    shooterT.set(-0.2);
    shooterB.set(-0.2);
  }

  public void Stop() {
    shooterT.set(0);
    shooterB.set(0);
  }
}
