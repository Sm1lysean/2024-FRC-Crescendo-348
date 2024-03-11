// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import java.util.List;
import frc.robot.subsystems.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //TalonSRX
  public static final TalonSRX Wrist = new TalonSRX(6);

  //Pneumatics
  public static final PneumaticHub PH = new PneumaticHub(1);

  // The robot's subsystems
  public static final DriveSubsystem rc_driveSS = new DriveSubsystem();
  public static final PIDSS rc_pidSS = new PIDSS();
  public static final WristPIDSS rc_WristPIDSS = new WristPIDSS();
  public static final ShooterSS rc_ShooterSS = new ShooterSS();
  public static final PneumaticsSS rc_pneumaticsSS = new PneumaticsSS();
  
  //Robots commands
  public static final ShooterControlC rc_ShooterControlC = new ShooterControlC(rc_ShooterSS, 0);
  public static final WristMovementC rc_WristMovementC = new WristMovementC(rc_WristPIDSS);
  public static final ClimberC rc_ClimberC = new ClimberC(rc_pneumaticsSS);
  public static final EndEffectorC rc_EndEffectorC = new EndEffectorC(rc_pneumaticsSS);
  public static final ZeroC rc_zeroC = new ZeroC(rc_pidSS);

  // The driver's controller
  public static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static final Trigger xButtonDr = new JoystickButton(m_driverController, XboxController.Button.kX.value); 
  public static final Trigger bButtonDr = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  public static final Trigger aButtonDr = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  public static final Trigger yButtonDr = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  public static final Trigger startButtonDr = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  public static final Trigger RightBumperDr = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  public static final Trigger LeftBumperDr = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  public static final POVButton dUpDr = new POVButton(m_driverController, 0);
  public static final POVButton dRightDr = new POVButton(m_driverController, 90);
  public static final POVButton dDownDr = new POVButton(m_driverController, 180);
  public static final POVButton dLeftDr = new POVButton(m_driverController, 270);

  //the operater's controller
  public static final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  public static final Trigger xButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kX.value); 
  public static final Trigger bButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  public static final Trigger aButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  public static final Trigger yButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  public static final Trigger startButtonOp = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);
  public static final Trigger RightBumperOp = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);
  public static final Trigger LeftBumperOp = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
  public static final POVButton dUpOp = new POVButton(m_operatorController, 0);
  public static final POVButton dRightOp = new POVButton(m_operatorController, 90);
  public static final POVButton dDownOp = new POVButton(m_operatorController, 180);
  public static final POVButton dLeftOp = new POVButton(m_operatorController, 270);

  // Cameras
  public static final UsbCamera camera1 = CameraServer.startAutomaticCapture("Shooter Camera", 0);
  public static final UsbCamera camera2 = CameraServer.startAutomaticCapture("Elevator Camera", 1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    rc_driveSS.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> rc_driveSS.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            rc_driveSS));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //Driver Controller
    startButtonDr.onTrue(new ZeroC());
    yButtonDr.whileTrue(rc_ShooterControlC);
    aButtonDr.onTrue(new ClimberC(rc_pneumaticsSS));
      
    //Operator Controller
    dUpOp.onTrue(new ElevatorC(rc_pidSS, () -> 450));
    dDownOp.onTrue(new ElevatorC(rc_pidSS, () -> 0));
    startButtonOp.whileTrue(new ZeroC());
    aButtonOp.onTrue(rc_WristMovementC);
    aButtonOp.onFalse(rc_WristMovementC);
    bButtonOp.onTrue(rc_WristMovementC);
    bButtonOp.onFalse(rc_WristMovementC);
    RightBumperOp.whileTrue(rc_ShooterControlC);
    LeftBumperOp.whileTrue(rc_ShooterControlC);
    yButtonOp.onTrue(new EndEffectorC(rc_pneumaticsSS));
    xButtonOp.onTrue((rc_ShooterControlC));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)), 
    List.of(new Translation2d(-5, 0)), new Pose2d(-5, 0, new 
    Rotation2d(180)), config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        rc_driveSS::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        rc_driveSS::setModuleStates,
        rc_driveSS);

    // Reset odometry to the starting pose of the trajectory.
    rc_driveSS.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> rc_driveSS.drive(0, 0, 0, false, false));
    }
}
