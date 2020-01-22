/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import java.io.*;

import javax.lang.model.util.ElementScanner6;

// Rev Spark Max classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// ultra sonic snsor classes
import edu.wpi.first.wpilibj.AnalogInput;

// add subsystems
import frc.robot.Climber;
import frc.robot.ControlPanel;
import frc.robot.Drive;
import frc.robot.Indexer;
import frc.robot.Intake;
import frc.robot.Shooter;

// network tables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  // create the subsystems
 
//  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  public OI m_OI = new OI();
  public Climber m_climber = new Climber();
  public ControlPanel m_controlPanel = new ControlPanel(); 
  public Drive m_drive = new Drive(m_OI.getDriverStick());
  public Indexer m_indexer = new Indexer();
  public Intake m_intake = new Intake();
  public Shooter m_shooter = new Shooter();

  private static final String UNKNOWN = "Unknown";
  private final Timer m_timer = new Timer();
  public CANSparkMax m_TestMotor = null;

 
  //setup ultra sonic sensor
  public AnalogInput ultrasonic0 = new AnalogInput(0);

  //network table
  private NetworkTable m_visionTable = null;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    // initialize the subsystems


    m_TestMotor = new CANSparkMax(11, MotorType.kBrushless);
    NetworkTableInstance networkTableInstance = NetworkTableInstance.create();
    networkTableInstance.startClient("10.32.1.105");
    System.out.println("Network Tables Connected? " + Boolean.toString(networkTableInstance.isConnected()));
    m_visionTable = networkTableInstance.getTable("ContourTable");
    m_drive.SetVisionTable(m_visionTable);
    m_OI.SetDrive(m_drive);


    

  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();


  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    //if (m_timer.get() < 2.0) {
    //  m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    //}//// else {
    //  m_robotDrive.stopMotor(); // stop robot
    //}
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    // call the periodic methods on all of the subsystems
    m_climber.periodic();
    m_controlPanel.periodic();
    m_drive.periodic();
    m_indexer.periodic();
    m_intake.periodic();
    m_shooter.periodic();
    m_OI.periodic();

    if (true)
      return;

    // sample/test code below

    //m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());

    // right trigger to spin forward
    /*
    double dValueLeft = m_stick.getRawAxis(2);
    double dValueRight = m_stick.getRawAxis(3);
    if (dValueRight != 0) {
      System.out.println("Right Trigger Value: " + Double.toString(dValueRight));
    }

    if (dValueRight > 0.25)
    {
      dValueRight = 0.25;
    } 

    
    // left trigger to spin backwards
    if (dValueLeft != 0) {
      System.out.println("Left Trigger Value: " + Double.toString(dValueLeft));
    }

    if (dValueLeft > 0.25)
    {
      dValueLeft = 0.25;
    }
    if (dValueLeft > 0.0) {
      m_TestMotor.set(-dValueLeft);
    }

    if (dValueRight > 0.0){
      m_TestMotor.set(dValueRight);
    } else if (dValueLeft > 0) {
      m_TestMotor.set(-dValueLeft);
    } else {
      m_TestMotor.set(0);
    }


  /*
  System.out.println("Color Value: " + colorString);
  System.out.println("Red Value: " + Double.toString(detectedColor.red)
                      + " Green Value: " + Double.toString(detectedColor.green)
                      + " Blue Value: " + Double.toString(detectedColor.blue));
  */                    
  
  //System.out.println("Ultrasonic: " + ultrasonic0.getValue());

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
