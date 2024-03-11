package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;

public class PIDSS extends SubsystemBase{
    
    TalonFX test = new TalonFX(0);
    public final CANSparkMax Elevator = new CANSparkMax(Constants.EL, MotorType.kBrushless);
    
    private final PIDController Velo_PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    private final PIDController Pos_PID = new PIDController(Constants.kP, Constants.kI, Constants.kD);

    private boolean manual = true;

    private double desiredVelocity;
    private double desiredPosition;

    private double voltage;

    public void ElevatorStop() {
        Elevator.set(0);
    }

    //Setup for the PID SS
    public PIDSS() {
        Elevator.setIdleMode(IdleMode.kBrake);
        Elevator.setInverted(true);
        Elevator.getEncoder().setPosition(0);
    }

    public void AutoElevator(double speed) {
        Elevator.set(speed);
    }
  
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Desired Elevator Velcity", desiredVelocity);
        SmartDashboard.putNumber("Voltage", getVoltage());

        setVoltage();
    }

    public void setVoltage(){
        double PIDVoltage;
        if (manual) {       //Calculate position, if it is the position, stop the arm
            PIDVoltage = Velo_PID.calculate(getVelocity(), desiredVelocity);
        } else {
            //Otherwise try to go to the position
            double PIDVelocity = Pos_PID.calculate(getPosition(), desiredPosition);
            PIDVoltage = Velo_PID.calculate(getVelocity(), PIDVelocity);
            //double feedforwardVoltage = feedforward.calculate(getPosition(), 10);
            //PIDVoltage = PIDVoltage + feedforwardVoltage;
        }

        voltage = PIDVoltage;

        Elevator.setVoltage(PIDVoltage);
    }

    public double getVoltage(){
        return voltage;
    }

    public void setVelocity(double degreesPerSecond) {
        manual = true;

        desiredVelocity = degreesPerSecond;
    }

    public void setPosition(double degrees) {
        manual = false;

        desiredPosition = degrees;
    }

    public double getPosition() {
        return Elevator.getEncoder().getPosition() * 8;
    }

    public double getVelocity() {
        return Elevator.getEncoder().getVelocity() * (2 / 15);
    }    
}
