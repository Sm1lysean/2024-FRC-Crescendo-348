package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristPIDSS extends SubsystemBase{

    public static final TalonSRX Wrist = new TalonSRX(Constants.Wrist);

    public void Spin(double speed) {
        Wrist.set(ControlMode.PercentOutput, speed);
    }

    public void Stop() {
        Wrist.set(ControlMode.PercentOutput, 0);
    }
    
}
