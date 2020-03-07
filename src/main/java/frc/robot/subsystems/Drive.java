/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.TheRobot;
import frc.robot.commands.Rumble;
import frc.robot.commands.ledColor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// network tables
import frc.robot.helper.ShooterLookUp;

public class Drive extends SubsystemBase {
  private  Joystick m_driverStick = null;
  private static DecimalFormat df3 = new DecimalFormat("#.###");
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(21);
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(22);

  private boolean m_bPowerPortTargeting = false;
  private boolean m_bPowerCellTargeting = false;
  private boolean m_bPowerPortTargetingAligned = false;
  private double m_dTargetMaxPower = 0.4;
  private double m_dTargetMinPower = 0.121;
  private double m_dTargetSpinP = 0.55;
  private double m_dTargetSpinDeadZone = 1.0;

  private ShooterLookUp m_lookUpTable = null; // look up table for shooter values

  private final static int kDriveStyle_tank = 0;
  private final static int kDriveStyle_arcade1 = 1;
  private final static int kDriveStyle_arcade2 = 2;
  private final static int kDriveStyle_arcade3 = 3;
  public int m_DriveStyle = Drive.kDriveStyle_arcade3;
  private DifferentialDrive m_differentialDrive = null;
  private double m_dOpenLoopRamp = 0.6;
  private boolean m_bUseJoystick = true;
  private boolean m_bSlowDrive = false;
  
  private ledColor m_LEDColor = ledColor.kNormal;

  /**
   * Creates a new Drive.
   */
  public Drive() {
    // setup talon FXs
    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_leftMotor.configOpenloopRamp(m_dOpenLoopRamp);
    m_rightMotor.configOpenloopRamp(m_dOpenLoopRamp);
    m_differentialDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_driverStick =  new Joystick(0); // drivers joystick


    SmartDashboard.putNumber("Drive/Max Power", m_dCurrentMaxPower);
    SmartDashboard.putNumber("Drive/Power Ramp Time", m_dOpenLoopRamp);


      SmartDashboard.putNumber("Drive/Drive Style", m_DriveStyle);
      SmartDashboard.putNumber("Targeting/Max Power", m_dTargetMaxPower);
      SmartDashboard.putNumber("Targeting/Min Power", m_dTargetMinPower);
      SmartDashboard.putNumber("Targeting/Spin P", m_dTargetSpinP);
      SmartDashboard.putNumber("Targeting/Spin Dead Zone", m_dTargetSpinDeadZone);
      SmartDashboard.putNumber("Targeting/Spin Steer", 0);

      m_lookUpTable = new ShooterLookUp();
  }

  private double m_deadzone = 0.05;
  private double m_dSlowMaxPower = 0.35;
  private double m_dFastMaxPower = 0.75;
  private double m_dCurrentMaxPower = 0.75;
  private int m_iLEDIndexFullCounter = 0;

  @Override
  public void periodic() {
    Robot r = TheRobot.getInstance();

    // This method will be called once per scheduler run

    // get/set drive style
    
    m_DriveStyle = (int)Math.round(SmartDashboard.getNumber("Drive/Drive Style", 0));
    m_dTargetMaxPower = SmartDashboard.getNumber("Targeting/Max Power", 0);
    m_dTargetMinPower = SmartDashboard.getNumber("Targeting/Min Power", 0);
    m_dTargetSpinP = SmartDashboard.getNumber("Targeting/Spin P", 0);
    m_dTargetSpinDeadZone = SmartDashboard.getNumber("Targeting/Spin Dead Zone", 0);
    SmartDashboard.putNumber("Drive/Left Encoder", m_leftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Drive/Right Encoder", m_rightMotor.getSelectedSensorPosition());


    m_dCurrentMaxPower = SmartDashboard.getNumber("Drive/Max Power", 0);
    double d = SmartDashboard.getNumber("Drive/Power Ramp Time", 0);

    if (d != m_dOpenLoopRamp) {
      m_dOpenLoopRamp = d;
      m_leftMotor.configOpenloopRamp(m_dOpenLoopRamp);
      m_rightMotor.configOpenloopRamp(m_dOpenLoopRamp);
    }

    if (!m_bSlowDrive) {
      m_LEDColor = ledColor.kNormal;
    } else {
      m_LEDColor = ledColor.kSlow;
    }

    // If the indexer is full
    // Only display for a second
    /*
    if (r.m_indexer.isFull() && m_iLEDIndexFullCounter < 50) {
      m_LEDColor = ledColor.kIndexerFull;
      m_iLEDIndexFullCounter++;
    } else if (!r.m_indexer.isFull()) { 
      m_iLEDIndexFullCounter = 0;
    }*/
   

    // drive the robot with the joysticks
    if (!m_bUseJoystick) {
      return;
    }

    //check if it says Left!
    double dvalueLXAxis = m_driverStick.getRawAxis(0);
    double dvalueLYAxis = -m_driverStick.getRawAxis(1); // joystick Y axis is inverted
    double dvalueRXAxis = m_driverStick.getRawAxis(4);
    double dvalueRYAxis = -m_driverStick.getRawAxis(5); // joystick Y axis is inverted

    // enforce deadzone on joysticks
    if (Math.abs(dvalueLXAxis) < m_deadzone) dvalueLXAxis = 0;
    if (Math.abs(dvalueLYAxis) < m_deadzone) dvalueLYAxis = 0;
    if (Math.abs(dvalueRXAxis) < m_deadzone) dvalueRXAxis = 0;
    if (Math.abs(dvalueRYAxis) < m_deadzone) dvalueRYAxis = 0;

    // enforce maximums on joysticks as a scale
      dvalueLYAxis = dvalueLYAxis * m_dCurrentMaxPower;
      dvalueLXAxis = dvalueLXAxis * m_dCurrentMaxPower;
      dvalueRYAxis = dvalueRYAxis * m_dCurrentMaxPower;
      dvalueRXAxis = dvalueRXAxis * m_dCurrentMaxPower;

    // drive the robot in manual mode
    if (!(m_bPowerPortTargeting || m_bPowerCellTargeting)) {
      this.JustDrive(dvalueLYAxis, dvalueRYAxis, dvalueLXAxis, dvalueRXAxis);


      
    } else if(m_bPowerPortTargeting) {
     this.PowerPortTargetDrive(dvalueLYAxis, dvalueRYAxis, dvalueLXAxis, dvalueRXAxis);
    }else{
    //this.PowerCellTargetDrive(0.2, dvalueRYAxis, dvalueLXAxis, dvalueRXAxis);
    }

    //r.m_LEDs.setColor(m_LEDColor);
    r.m_LEDs.setColor(m_LEDColor);
  }

  public void enableBrakes(boolean enable) {
    if (enable) {
      m_leftMotor.setNeutralMode(NeutralMode.Brake);
      m_rightMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_leftMotor.setNeutralMode(NeutralMode.Coast);
      m_rightMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  private void PowerPortTargetDrive(double dvalueLYAxis, double dvalueRYAxis, double dvalueLXAxis, double dvalueRXAxis) {
    double targetAngle = m_lookUpTable.getTargetAngle();
    double dFrame = m_lookUpTable.getFrameCounter();

    if (true) {
    TheRobot.log("Frame: " +df3.format(dFrame) +
                 " TargetAngle: " + df3.format(targetAngle));
    }

    // If the target isn't found let the driver know
    Robot r = TheRobot.getInstance();
    if (!m_lookUpTable.isTargetFound()) {
      //CommandBase c = new Rumble(r.getDriverStick(), RumbleType.kLeftRumble);
      //r.m_CMDScheduler.schedule(c.withTimeout(1.0));
      m_LEDColor = ledColor.kTargetNotFound;
    } else {
      m_LEDColor = ledColor.kTargetFound;
    }

    if (dvalueLYAxis == 0 && dvalueRYAxis == 0) {
      this.TargetDriveSpin(targetAngle, dFrame);
      return;
    }

    // target while driving
    double steer = 0;
    switch (m_DriveStyle) {
      case Drive.kDriveStyle_tank: {
        // Turn right?
        if (targetAngle > 0) dvalueRYAxis = 0;

        // Turn left?
        if (targetAngle < 0) dvalueLYAxis = 0;
        TheRobot.log("PowerPortTargetDrive: kDriveStyle_tank");
        m_differentialDrive.tankDrive(dvalueLYAxis, dvalueRYAxis);
      } break;
      case Drive.kDriveStyle_arcade2:
        dvalueLYAxis = dvalueRYAxis;
        
      case Drive.kDriveStyle_arcade1:
      case Drive.kDriveStyle_arcade3:
        // scaling steer to -1 to 1 over 45 degrees
        steer = targetAngle/45.0;
        if (steer > 1.0) steer = 1.0;
        if (steer < -1.0) steer = -1.0;
        TheRobot.log("PPortSpeed: " + TheRobot.toString(dvalueLYAxis) 
        + " Steer: " + TheRobot.toString(steer));
        m_differentialDrive.arcadeDrive(dvalueLYAxis, steer, true);
        break;
    }
  }

  public void PowerCellTargetDrive(double dvalueLYAxis, double dvalueRYAxis, double dvalueLXAxis, double dvalueRXAxis) {
    Robot r = TheRobot.getInstance();
    double targetAngle = r.m_PCTargeter.getPowerCellAngle();
    double dFrame = r.m_PCTargeter.getPowerCellFrameCounter();

    if (true) {
    TheRobot.log("Frame: " +df3.format(dFrame) +
                 " TargetAngle: " + df3.format(targetAngle));
    }

    // If the target isn't found let the driver know
    if (!r.m_PCTargeter.isPowerCellFound()) {
      m_LEDColor = ledColor.kTargetNotFound;
    } else {
      m_LEDColor = ledColor.kTargetFound;
    }

    if (dvalueLYAxis == 0 && dvalueRYAxis == 0) {
      this.TargetDriveSpin(targetAngle, dFrame);
      return;
    }

    // target while driving
    double steer = 0;
    switch (m_DriveStyle) {
      case Drive.kDriveStyle_tank: {
        // Turn right?
        if (targetAngle > 0) dvalueRYAxis = 0;

        // Turn left?
        if (targetAngle < 0) dvalueLYAxis = 0;
        TheRobot.log("PowerCellTargetDrive: kDriveStyle_tank");

        m_differentialDrive.tankDrive(dvalueLYAxis, dvalueRYAxis);
      } break;
      case Drive.kDriveStyle_arcade2:
        dvalueLYAxis = dvalueRYAxis;
        
      case Drive.kDriveStyle_arcade1:
      case Drive.kDriveStyle_arcade3:
        // scaling steer to -1 to 1 over 45 degrees
        steer = targetAngle/45.0;
        if (steer > 1.0) steer = 1.0;
        if (steer < -1.0) steer = -1.0;

        if(Math.abs(steer) > 0.1){
          if(steer > 0) dvalueLYAxis = 0;
          if(steer < 0) dvalueRYAxis = 0;
        }

        TheRobot.log("PCSpeed: " + TheRobot.toString(dvalueLYAxis) 
            + " Steer: " + TheRobot.toString(steer));
        NudgeDrive(dvalueLYAxis, dvalueRYAxis);//.arcadeDrive(dvalueLYAxis, steer, true);
        break;
    }
  }

  private double m_dLastFrame = 0; // keep track of the previous frame processed by vision
  public void TargetDriveSpin(double targetAngle, double frame) {
    Robot r = TheRobot.getInstance();

    double steer = targetAngle/45.0;

    if (Math.abs(targetAngle) <= m_dTargetSpinDeadZone){
      TheRobot.log("Spin Speed stopped.");
      m_differentialDrive.arcadeDrive(0, 0, false);
      steer = 0;
      SmartDashboard.putNumber("Targeting/Spin Steer", steer);
      m_bPowerPortTargetingAligned = true;
      if (m_lookUpTable.isTargetFound()) m_LEDColor = ledColor.kOnTarget;
      return;
    } else {
      m_bPowerPortTargetingAligned = false;
    }

    // if we don't have any new information don't do anything new
    // so just return
    if (m_dLastFrame == frame) {
      return;
    } else {
      m_dLastFrame = frame;
    }

    

    if (steer > m_dTargetMaxPower) steer = m_dTargetMaxPower;
    if (steer < -m_dTargetMaxPower) steer = -m_dTargetMaxPower;

    steer = steer * m_dTargetSpinP;

    // use dNudge to slowly back robot up if angle is very small
    double dNudge = 0;
    if (Math.abs(steer) < m_dTargetMinPower) {
      if (steer < 0) steer = -m_dTargetMinPower;
      if (steer > 0) steer = m_dTargetMinPower;
      dNudge = Math.abs(m_dTargetMinPower)/2.0;
    }

    TheRobot.log("Spin Speed: " + TheRobot.toString(-dNudge) 
    + " Steer: " + TheRobot.toString(steer));
    m_differentialDrive.arcadeDrive(-dNudge, steer, false);
    SmartDashboard.putNumber("Targeting/Spin Steer", steer);
  }

  // sets the drive power for the left & right motors
  // corrects for the orientation of the mount
  private void JustDrive(double dvalueLYAxis, double dvalueRYAxis, double dvalueLXAxis, double dvalueRXAxis) {
    //TheRobot.log("JustDrive Velocities" + TheRobot.toString(dvalueLYAxis) + ", " + TheRobot.toString(dvalueRYAxis));

    switch (m_DriveStyle) {
      case Drive.kDriveStyle_tank:
      TheRobot.log("JustDrive: kDriveStyle_tank");
        m_differentialDrive.tankDrive(dvalueLYAxis, dvalueRYAxis);
        break;
      case Drive.kDriveStyle_arcade1:
      TheRobot.log("JustDrive: kDriveStyle_arcade1");
        m_differentialDrive.arcadeDrive(dvalueLYAxis, dvalueLXAxis);
        break;
      case Drive.kDriveStyle_arcade2:
        TheRobot.log("JustDrive: kDriveStyle_arcade2");

        m_differentialDrive.arcadeDrive(dvalueRYAxis, dvalueLXAxis);
        break;
      case Drive.kDriveStyle_arcade3:
      TheRobot.log("Speed: " + TheRobot.toString(dvalueLYAxis) 
            + " Steer: " + TheRobot.toString(dvalueRXAxis));
        m_differentialDrive.arcadeDrive(dvalueLYAxis, dvalueRXAxis, true);
        break;
    }
  }

  public void moveAtVelocity(double LeftVelocity, double RightVelocity)
  {
    TheRobot.log("Auto Velocities" + TheRobot.toString(LeftVelocity));
    m_differentialDrive.arcadeDrive(LeftVelocity, 0);
    //m_differentialDrive.tankDrive(LeftVelocity, RightVelocity);
    //JustDrive(LeftVelocity, 0, 0, 0);
  }


  public void SetPowerPortTargeting(boolean b) {
    m_bPowerPortTargetingAligned = false;
    m_bPowerPortTargeting = b;
  }

  public void SetPowerCellTargeting(boolean b) {
    m_bPowerCellTargeting = false;
    m_bPowerCellTargeting = b;
  }

  public boolean GetTargetingAligned() {
    return m_bPowerPortTargetingAligned;
  }
  
  public boolean GetTargeting() {
    return (m_bPowerPortTargeting || m_bPowerCellTargeting);
  }

public double getLeftEncoderPosition()
{
  return m_leftMotor.getSelectedSensorPosition();
}

public double getRightEncoderPosition()
{
  return -m_rightMotor.getSelectedSensorPosition();
}
  //check to see if the intake arm is retracted 
  //
  public boolean retract() {
    return false;
  }

  public void SetUseJoystick(boolean b) {
    m_bUseJoystick = b;
  }

  public void ToggleSlowDrive() {
    m_bSlowDrive = !m_bSlowDrive;

    if (m_bSlowDrive) {
      m_dCurrentMaxPower = m_dSlowMaxPower;
    } else {
      m_dCurrentMaxPower = m_dFastMaxPower;
    }
  }
/*
  public void resetEncoders() {
    m_.setPosition(0);
    m_encoderBottom.setPosition(0);
  }
  */

  public void NudgeDrive(double left, double right) {
    //m_differentialDrive.tankDrive(left, right);
    TheRobot.log("NudgeDrive.");
    m_leftMotor.set(left);
    m_rightMotor.set(-right);
  }
}
