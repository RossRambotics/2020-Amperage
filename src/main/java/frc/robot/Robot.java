/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// add subsystems
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PowerCellTargeter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Stick;
import frc.robot.subsystems.LEDController;
import frc.robot.subsystems.Hood;
import frc.robot.auto.AutoTarget;
import frc.robot.auto.DriveStraight;
import frc.robot.auto.DriveStriaghtWEncoders;
import frc.robot.commands.*;



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
  public OI m_OI = null;
  public Climber m_climber = null;
  public ControlPanel m_controlPanel = null;
  public Drive m_drive = null;
  public Indexer m_indexer = null;
  public Intake m_intake = null;
  public Shooter m_shooter = null;
  public Hood m_hood = null;
  public LEDController m_LEDs = null;
  public PowerCellTargeter m_PCTargeter = null;
  public CommandScheduler m_CMDScheduler = null;

  private static final String UNKNOWN = "Unknown";

  private final Timer m_timer = new Timer();
  private Stick m_DriverStick = null;
 

  public Robot() {
    TheRobot.SetInstance(this);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    TheRobot.log("robotInit.");

    m_climber = new Climber();
    m_controlPanel = new ControlPanel(); 
    m_drive = new Drive();
    m_intake = new Intake();
    m_indexer = new Indexer();
    m_shooter = new Shooter();
    m_hood = new Hood();
    m_PCTargeter = new PowerCellTargeter();
    m_LEDs = new LEDController();


    // Setup the singleton for easy access to the robot and subsystems
    m_robotContainer = new RobotContainer();
    m_DriverStick = new Stick(m_robotContainer.getDriverStick());

    // Get the scheduler
    m_CMDScheduler = CommandScheduler.getInstance();

    // initialize the subsystems
  
    // add commands to Dashboard
    CommandBase c;
    SmartDashboard.putData("Commands/Shoot!", new Shoot(m_indexer));
    SmartDashboard.putData("Commands/Retract Intake!", new RetractIntake());    
    SmartDashboard.putData("Commands/Extend Intake!", new ExtendIntake(m_indexer));
    SmartDashboard.putData("Commands/Indexer/CheckForNewPC!", new IndexerCheckForNewPowerCell(m_indexer));
    SmartDashboard.putData("Commands/Indexer/IndexNewPC!", new IndexNewPowerCell());
    SmartDashboard.putData("Commands/Indexer/CompactIndexer!", new CompactIndexer(m_indexer));
    SmartDashboard.putData("Commands/Indexer/CompactShooter!", new CompactShooter().withTimeout(0.2));

    SmartDashboard.putNumber("Auto/Move Distance", 0.0);

    /* 
     *
     * Begin autonomous setup of commands 
     * 
     */
    SmartDashboard.getEntry("Auto Mode/Selector").setString("0");  //SendableChooser for autocommand

    /* 
     * End autonomous command groups
     * 
     */

    c = new SequentialCommandGroup(
      new AutoTarget(m_drive).withTimeout(3.0),
      new Shoot(m_indexer).withTimeout(5.0)
    );

    c.setName("Auto Target & Shoot");
    SmartDashboard.putData("Commands/Auto/Auto Target & Shoot!", c);
    
    c = new ReverseCompactIndexer(m_indexer).withTimeout(0.5);
    c.setName("Reverse Compact");
    SmartDashboard.putData("Commands/Indexer/RevCompactIndexer!", c);

    c = new Rumble(m_DriverStick, RumbleType.kLeftRumble).withTimeout(1.0);
    c.setName("Rumble");
    SmartDashboard.putData("Commands/Rumble !", c);

    c = new ParallelRaceGroup(
      new ClearIndexer(m_indexer).withTimeout(0.1),
      new CompactShooter().withTimeout(0.1)
    );
    c.setName("Prepare to Shoot!");
    SmartDashboard.putData("Commands/Shooter/Prepare!", c); 

    c = new SequentialCommandGroup(
      new RetractIntake(), 
      new WaitCommand(0.25), 
      new ExtendIntake(m_indexer),
      new CompactIndexer(m_indexer),
      new WaitCommand(0.25)
      );

      c.setName("Smash & Compact");
      SmartDashboard.putData("Commands/Indexer/Smash & Compact!", c); 

    // 1 meter, 0.2 power
    c = new DriveStriaghtWEncoders(m_drive, 1, .2).withTimeout(2);
    c.setName("TestDriveStraight");
    SmartDashboard.putData("Commands/Drive/TestDriveStraight", c);
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();

    
    double autoMoveDistance = SmartDashboard.getNumber("Auto/Move Distance", 0.0);
    // the distance to move with the appriate auto command
    String s = SmartDashboard.getEntry("Auto Mode/Selector").getString("0");// the automode to use
    
    TheRobot.log("Autonomous Mode: " +s);
    
    CommandBase c = null;
    switch (s) {

      case "0":
        /*
        * Aim & Shoot!
        */
        c = new SequentialCommandGroup(
        new AutoTarget(m_drive).withTimeout(3.0),
        new Shoot(m_indexer).withTimeout(5.0)
        );
      break;
      case "1":
        /*
        * Aim, Shoot & Drive Forward 1m
        */
        c = new SequentialCommandGroup(
          new AutoTarget(m_drive).withTimeout(3.0),
          new Shoot(m_indexer).withTimeout(5.0),
          new DriveStraight(1.0, 0.5)
        );
      break;
      case "2":
        /*
        * Aim, Shoot & Drive Backward 1m
        */
        c = new SequentialCommandGroup(
        new AutoTarget(m_drive).withTimeout(3.0),
        new Shoot(m_indexer).withTimeout(5.0),
        new DriveStraight(1.0, -0.5)
        );
      break;
      case "3":
      default:
    }

    if (c != null) {
      c.schedule();
      m_autonomousCommand = c;
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
 
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    
    if(m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {

    // call the periodic methods on all of the subsystems
    /*
    m_climber.periodic();
    m_controlPanel.periodic();
    m_drive.periodic();
    m_indexer.periodic();
    m_shooter.periodic();
    m_OI.periodic();
    m_hood.periodic();
    m_intake.periodic();
    */


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

  public Stick getDriverStick() {
    return m_DriverStick;
  }

}
