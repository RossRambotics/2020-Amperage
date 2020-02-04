/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.*;

import javax.lang.model.util.ElementScanner6;

// Rev Spark Max classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;

// ultra sonic snsor classes
import edu.wpi.first.wpilibj.AnalogInput;

// add subsystems
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.PowerPortTargeter;
import frc.robot.commands.Shoot;

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
  private Command m_autonomousCommand = null;
  private RobotContainer m_robotContainer = null;

  // create the subsystems
  public OI m_OI = new OI();
  public Climber m_climber = new Climber();
  public ControlPanel m_controlPanel = new ControlPanel(); 
  public Drive m_drive = new Drive(m_OI.getDriverStick());
  public Indexer m_indexer = new Indexer();
  public Intake m_intake = new Intake();
  public Shooter m_shooter = new Shooter();
  public PowerPortTargeter m_powerPowerTargeter = new PowerPortTargeter();
  public CommandScheduler m_CMDScheduler = null;

  private static final String UNKNOWN = "Unknown";
  private final Timer m_timer = new Timer();
  public CANSparkMax m_TestMotor = null;
  private CANSparkMax m_leftMotor = null;
  private CANSparkMax m_rightMotor = null;
  private CANSparkMax m_indexerbottomMotor = null;
  private CANSparkMax m_indexertopMotor = null;
  private Joystick m_stick = null;
  private CANEncoder m_leftEncoder = null;
  private CANEncoder m_rightEncoder = null;

  // setup ultra sonic sensor
  public AnalogInput ultrasonic0 = new AnalogInput(0);

  // network table
  private NetworkTable m_visionTable = null;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    TheRobot.log("robotInit.");
    m_robotContainer = new RobotContainer();

    // Setup the singleton for easy access to the robot and subsystems
    TheRobot.SetInstance(this);

    // Get the scheduler
    m_CMDScheduler = CommandScheduler.getInstance();

    // initialize the subsystems
    m_TestMotor = new CANSparkMax(30, MotorType.kBrushless);
    final NetworkTableInstance networkTableInstance = NetworkTableInstance.create();
    //networkTableInstance.startClient("10.32.1.105");
    // System.out.println("Network Tables Connected? " + Boolean.toString(networkTableInstance.isConnected()));
    m_visionTable = networkTableInstance.getTable("ContourTable");
    m_drive.SetVisionTable(m_visionTable);
    m_OI.SetDrive(m_drive);
    m_indexerbottomMotor = new CANSparkMax(30, MotorType.kBrushless);
    m_indexertopMotor = new CANSparkMax(30, MotorType.kBrushless);
    m_leftMotor = new CANSparkMax(30, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(30, MotorType.kBrushless);
    m_stick = new Joystick(0);
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();

    // add commands to Dashboard
    SmartDashboard.putData("Shoot!", new Shoot());
    
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

  @Override
  public void robotPeriodic() {

    m_CMDScheduler.run();

    // Auto-generated method stub
    super.robotPeriodic();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

}
