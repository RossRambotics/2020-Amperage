/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TheRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// network tables
import edu.wpi.first.networktables.NetworkTable;

public class Drive extends SubsystemBase {
  private  Joystick m_driverStick = null;
  private static DecimalFormat df3 = new DecimalFormat("#.###");
  private final WPI_TalonFX m_leftMotor = new WPI_TalonFX(21);
  private final WPI_TalonFX m_rightMotor = new WPI_TalonFX(22);
  private boolean m_bTargeting = false;
  private NetworkTable m_visionTable = null;

  private final static int kDriveStyle_tank = 0;
  private final static int kDriveStyle_arcade1 = 1;
  private final static int kDriveStyle_arcade2 = 2;
  private final static int kDriveStyle_arcade3 = 3;
  private int m_DriveStyle = Drive.kDriveStyle_tank;
  private DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  


  /**
   * Creates a new Drive.
   */
  public Drive(Joystick j) {
      m_driverStick = j;

      SmartDashboard.putNumber("Drive\\Drive Style", m_DriveStyle);
  }

  public void SetVisionTable(NetworkTable n) {
    m_visionTable = n;
  }

  public NetworkTable GetNetworkTable() {
    return m_visionTable;
  }

  private double m_deadzone = 0.05;
  private double m_maxPower = 1.00;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // get/set drive style
    
    m_DriveStyle = (int)Math.round(SmartDashboard.getNumber("Drive\\Drive Style", 0));
    TheRobot.log("Driver style: " + m_DriveStyle);

    // drive the robot with the joysticks

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

  private void TargetDrive(double dvalueLYAxis, double dvalueRYAxis, double dvalueLXAxis, double dvalueRXAxis) {
    double targetAngle = m_visionTable.getEntry("TargetAngle").getDouble(0);
    double dFrame = m_visionTable.getEntry("FrameCounter").getDouble(0);

    if (true) {
    //System.out.println("Frame: " +df3.format(dFrame) +
   //                   " TargetAngle: " + df3.format(dDirection));
    }

    if (dvalueLYAxis == 0 && dvalueRYAxis == 0) {
      this.TargetDriveSpin(targetAngle, dFrame);
      return;
    }

    // target while driving
    switch (m_DriveStyle) {
      case Drive.kDriveStyle_tank: {
        // Turn right?
        if (targetAngle > 0) dvalueRYAxis = 0;

        // Turn left?
        if (targetAngle < 0) dvalueLYAxis = 0;

        m_differentialDrive.tankDrive(dvalueLYAxis, dvalueRYAxis);
      } break;
      case Drive.kDriveStyle_arcade1:
        m_differentialDrive.arcadeDrive(dvalueLYAxis, dvalueLXAxis);
        break;
      case Drive.kDriveStyle_arcade2:
        m_differentialDrive.arcadeDrive(dvalueRYAxis, dvalueLXAxis);
        break;
       case Drive.kDriveStyle_arcade3:
        m_differentialDrive.arcadeDrive( dvalueLYAxis, dvalueRXAxis);
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

  private void TargetDriveSpin(double targetAngle, double frame) {
    if (targetAngle > 5) {
      // turn right
      //this.JustDrive(1,-1);
    } else if (targetAngle < -5) {
      // turn left
      //this.JustDrive(-1,1);
    } else {
      // stop turning
      //this.JustDrive(0, 0);
    }
  }

  // sets the drive power for the left & right motors
  // corrects for the orientation of the mount
  private void JustDrive(double dvalueLYAxis, double dvalueRYAxis, double dvalueLXAxis, double dvalueRXAxis) {
    switch (m_DriveStyle) {
      case Drive.kDriveStyle_tank:
        m_differentialDrive.tankDrive(dvalueLYAxis, dvalueRYAxis);
        break;
      case Drive.kDriveStyle_arcade1:
        TheRobot.log("In arcade drive.");
        m_differentialDrive.arcadeDrive(dvalueLYAxis, dvalueLXAxis);
        break;
      case Drive.kDriveStyle_arcade2:
        m_differentialDrive.arcadeDrive(dvalueRYAxis, dvalueLXAxis);
        break;
      case Drive.kDriveStyle_arcade3:
        m_differentialDrive.arcadeDrive(dvalueLYAxis, dvalueRXAxis,true);
        break;
    }
  }


  public void SetTargeting(boolean b) {
    m_bTargeting = b;
  }

  //check to see if the intake arm is retracted 
  //
  public boolean retract() {
    return false;
  }
}
