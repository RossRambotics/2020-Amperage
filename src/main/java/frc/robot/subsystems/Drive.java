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



  private boolean m_bTargeting = false;
  private boolean m_bTargetingAligned = false;
  private double m_dTargetMaxPower = 0.4;
  private double m_dTargetMinPower = 0.1;
  private double m_dTargetSpinP = 0.6;
  private double m_dTargetSpinDeadZone = 1.0;

  private ShooterLookUp m_lookUpTable = null; // look up table for shooter values

  private final static int kDriveStyle_tank = 0;
  private final static int kDriveStyle_arcade1 = 1;
  private final static int kDriveStyle_arcade2 = 2;
  private final static int kDriveStyle_arcade3 = 3;
  private int m_DriveStyle = Drive.kDriveStyle_arcade3;
  private DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private double m_dOpenLoopRamp = 0.6;
  private boolean m_bUseJoystick = true;
  


  /**
   * Creates a new Drive.
   */
  public Drive() {
    // setup talon FXs
    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_leftMotor.configOpenloopRamp(m_dOpenLoopRamp);
    m_rightMotor.configOpenloopRamp(m_dOpenLoopRamp);

    m_driverStick =  new Joystick(0);

    SmartDashboard.putNumber("Drive/Max Power", m_maxPower);
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
  private double m_maxPower = 0.75;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // get/set drive style
    
    m_DriveStyle = (int)Math.round(SmartDashboard.getNumber("Drive/Drive Style", 0));
    m_dTargetMaxPower = SmartDashboard.getNumber("Targeting/Max Power", 0);
    m_dTargetMinPower = SmartDashboard.getNumber("Targeting/Min Power", 0);
    m_dTargetSpinP = SmartDashboard.getNumber("Targeting/Spin P", 0);
    m_dTargetSpinDeadZone = SmartDashboard.getNumber("Targeting/Spin Dead Zone", 0);

    m_maxPower = SmartDashboard.getNumber("Drive/Max Power", 0);
    double d = SmartDashboard.getNumber("Drive/Power Ramp Time", 0);

    if (d != m_dOpenLoopRamp) {
      m_dOpenLoopRamp = d;
      m_leftMotor.configOpenloopRamp(m_dOpenLoopRamp);
      m_rightMotor.configOpenloopRamp(m_dOpenLoopRamp);
    }


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

    // enforce maximums on joysticks
    if (Math.abs(dvalueLYAxis) > m_maxPower) {
      if (dvalueLYAxis > 0) dvalueLYAxis = m_maxPower; else dvalueLYAxis = -m_maxPower;
    }
    if (Math.abs(dvalueLXAxis) > m_maxPower) {
      if (dvalueLXAxis > 0) dvalueLXAxis = m_maxPower; else dvalueLXAxis = -m_maxPower;
    }
    if (Math.abs(dvalueRYAxis) > m_maxPower) {
      if (dvalueRYAxis > 0) dvalueRYAxis = m_maxPower; else dvalueRYAxis = -m_maxPower;
    }
    if (Math.abs(dvalueRXAxis) > m_maxPower) {
      if (dvalueRXAxis > 0) dvalueRXAxis = m_maxPower; else dvalueRXAxis = -m_maxPower;
    }

    // drive the robot in manual mode
    if (!m_bTargeting) {
      this.JustDrive(dvalueLYAxis, dvalueRYAxis, dvalueLXAxis, dvalueRXAxis);
    } else {
      this.TargetDrive(dvalueLYAxis, dvalueRYAxis, dvalueLXAxis, dvalueRXAxis);
    }
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

  private void TargetDrive(double dvalueLYAxis, double dvalueRYAxis, double dvalueLXAxis, double dvalueRXAxis) {
    double targetAngle = m_lookUpTable.getTargetAngle();
    double dFrame = m_lookUpTable.getFrameCounter();

    if (true) {
    TheRobot.log("Frame: " +df3.format(dFrame) +
                 " TargetAngle: " + df3.format(targetAngle));
    }

    // If the target isn't found let the driver know
    if (!m_lookUpTable.isTargetFound()) {
      Robot r = TheRobot.getInstance();
      CommandBase c = new Rumble(r.getDriverStick(), RumbleType.kLeftRumble);
      r.m_CMDScheduler.schedule(c.withTimeout(1.0));
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
        m_differentialDrive.arcadeDrive(dvalueLYAxis, steer);
        break;
    }

    if (targetAngle > 0) {
      // turn right
      //this.JustDrive(dvalueLYAxis,0);
    } else if (targetAngle < 0) {
      // turn left
      //this.JustDrive(0, dvalueRYAxis);
    } else {
      // stop turning
      //this.JustDrive(0, 0);
    }

  }

  private double m_dLastFrame = 0; // keep track of the previous frame processed by vision
  public void TargetDriveSpin(double targetAngle, double frame) {
    if (Math.abs(targetAngle) <= m_dTargetSpinDeadZone){
      m_differentialDrive.arcadeDrive(0, 0, false);
      m_bTargetingAligned = true;
    } else {
      m_bTargetingAligned = false;
    }

    // if we don't have any new information don't do anything new
    // so just return
    if (m_dLastFrame == frame) {
      return;
    } else {
      m_dLastFrame = frame;
    }

    double steer = targetAngle/45.0;

    if (steer > m_dTargetMaxPower) steer = m_dTargetMaxPower;
    if (steer < -m_dTargetMaxPower) steer = -m_dTargetMaxPower;

    steer = steer * m_dTargetSpinP;

    if (Math.abs(steer) < m_dTargetMinPower) {
      if (steer < 0) steer = -m_dTargetMinPower;
      if (steer > 0) steer = m_dTargetMinPower;
    }

    m_differentialDrive.arcadeDrive(0, steer, false);
    SmartDashboard.putNumber("Targeting/Spin Steer", steer);
  }

  // sets the drive power for the left & right motors
  // corrects for the orientation of the mount
  private void JustDrive(double dvalueLYAxis, double dvalueRYAxis, double dvalueLXAxis, double dvalueRXAxis) {
    //TheRobot.log("JustDrive Velocities" + TheRobot.toString(dvalueLYAxis) + ", " + TheRobot.toString(dvalueRYAxis));

    switch (m_DriveStyle) {
      case Drive.kDriveStyle_tank:
        m_differentialDrive.tankDrive(dvalueLYAxis, dvalueRYAxis);
        break;
      case Drive.kDriveStyle_arcade1:
        m_differentialDrive.arcadeDrive(dvalueLYAxis, dvalueLXAxis);
        break;
      case Drive.kDriveStyle_arcade2:
        m_differentialDrive.arcadeDrive(dvalueRYAxis, dvalueLXAxis);
        break;
      case Drive.kDriveStyle_arcade3:
        m_differentialDrive.arcadeDrive(dvalueLYAxis, dvalueRXAxis, true);
        break;
    }
  }

  public void moveAtVelocity(double LeftVelocity, double RightVelocity)
  {
    //TheRobot.log("Auto Velocities" + TheRobot.toString(LeftVelocity));
    m_leftMotor.set(LeftVelocity);
    m_rightMotor.set(LeftVelocity); 
    //m_differentialDrive.tankDrive(LeftVelocity, RightVelocity);
    //JustDrive(LeftVelocity, 0, 0, 0);
  }


  public void SetTargeting(boolean b) {
    m_bTargetingAligned = false;
    m_bTargeting = b;
  }

  public boolean GetTargetingAligned() {
    return m_bTargetingAligned;
  }
  
  public boolean GetTargeting() {
    return m_bTargeting;
  }

public double getLeftEncoderPosition()
{
  return m_leftMotor.getSelectedSensorPosition();
}

public double getRightEncoderPosition()
{
  return m_rightMotor.getSelectedSensorPosition();
}
  //check to see if the intake arm is retracted 
  //
  public boolean retract() {
    return false;
  }

  public void SetUseJoystick(boolean b) {
    m_bUseJoystick = b;
  }
/*
  public void resetEncoders() {
    m_.setPosition(0);
    m_encoderBottom.setPosition(0);
  }
  */
}
