// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Console;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends TimedRobot {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private double autoturnspeed = 0.5;
  private double autospeed = 0.5;
  private LimelightHelpers limelight;
  private double slowdownamount = 0;
  private boolean bDown = false;
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
    
    LimelightHelpers.setLEDMode_PipelineControl("limelight");
    LimelightHelpers.setLEDMode_PipelineControl("limelight");

    LimelightHelpers.setLEDMode_PipelineControl("limelight");

    LimelightHelpers.setLEDMode_PipelineControl("limelight");

    LimelightHelpers.setLEDMode_ForceBlink("limelight");
    m_timer.restart();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
      
      if (Math.abs(x) > 5) {
        if (Math.signum(x) == -1){
          //m_drive.tankDrive(autoturnspeed, autoturnspeed);
        }
        if (Math.signum(x) == 1){
          //m_drive.tankDrive(-autoturnspeed, -autoturnspeed);
        }
        
      } else {
        //m_drive.tankDrive(autospeed, -autospeed);
      }
  }
  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double x = Math.signum(m_controller.getRightX())*((m_controller.getRightX()*m_controller.getRightX()));
    double y = m_controller.getLeftY()*-1;
    
    if (Math.abs(x) <= 0.4){x = 0;}
    if (Math.abs(y) <= 0.4){y = 0;}
    slowdownamount = 1-(0.8 * m_controller.getLeftTriggerAxis());
    System.out.println(slowdownamount);
    /*(-x/2) + (y/2),  (-x/2) - (y/2) */
    if (m_controller.getBButtonPressed()){
      bDown = true;
    }
    
    else if(m_controller.getAButtonReleased()){
      bDown = false;
    }
    if (bDown){
      m_drive.tankDrive(((-x/2) + (y/2))*2*slowdownamount,  ((-x/2) - (y/2))*2*slowdownamount);

    }
    else{
      m_drive.tankDrive(((-x/2) + (y/2))*slowdownamount, ((-x/2) - (y/2))*slowdownamount);
    }
    m_controller.setRumble(RumbleType.kRightRumble, Math.abs(y));
    m_controller.setRumble(RumbleType.kLeftRumble, Math.abs(x));
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
