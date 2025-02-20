// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.CANRollerSubsystem;
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html USEW
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

 // The motors on the left side of the drive.
 private final SparkMax m_leftLeader = new SparkMax(4, MotorType.kBrushed);
 private final SparkMax m_leftFollower = new SparkMax(3, MotorType.kBrushed);

 // The motors on the right side of the drive.
 private final SparkMax m_rightLeader = new SparkMax(5, MotorType.kBrushed);
 private final SparkMax m_rightFollower = new SparkMax(2, MotorType.kBrushed);

 
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  // The robot's drive
  private final DifferentialDrive m_drive;
  private final CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();


  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(8);
  private final SparkMaxConfig m_dConfig = new SparkMaxConfig();

  // Odometry class for tracking robot pose

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_dConfig.follow(m_leftLeader);
    m_leftFollower.configure(m_dConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_dConfig.follow(m_rightLeader);
    m_rightFollower.configure(m_dConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_dConfig.disableFollowerMode();
    m_rightLeader.configure(m_dConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftLeader.configure(m_dConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  }

  @Override
  public void robotPeriodic(){
  }
  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
  
    } else {
      m_drive.stopMotor(); // stop robot
    }
  }
  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double x = Math.signum(m_controller.getLeftX())*(m_controller.getLeftX()*m_controller.getLeftX());
    double y = m_controller.getLeftY()*-1;


    if (Math.abs(x) <= 0.4){x = 0;}
    if (Math.abs(y) <= 0.4){y = 0;}
    /*(-x/2) + (y/2),  (-x/2) - (y/2) */
    m_drive.tankDrive((-x/2) + (y/2),  (-x/2) - (y/2));
    // ill add ts input https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/XboxController.html
    if (m_controller.getYButton()) {
      rollerSubsystem.runRoller(1.0, 0.0);
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
