/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// network tables
import edu.wpi.first.networktables.NetworkTable;

public class Drive extends SubsystemBase {
  private  Joystick m_driverStick = null;
  private static DecimalFormat df3 = new DecimalFormat("#.###");
  private final TalonSRX m_leftMotor = new TalonSRX(1);
  private final TalonSRX m_rightMotor = new TalonSRX(2);
  private boolean m_bTargeting = false;
  private NetworkTable m_visionTable = null;

  /**
   * Creates a new Drive.
   */
  public Drive(Joystick j) {
      m_driverStick = j;
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


    // drive the robot with the joysticks

    //check if it says Left!
    double dvalueLYAxis = m_driverStick.getRawAxis(1);
    double dvalueRYAxis = -m_driverStick.getRawAxis(5);

    // enforce deadzone on joysticks
    if (Math.abs(dvalueLYAxis) < m_deadzone) dvalueLYAxis = 0;
    if (Math.abs(dvalueRYAxis) < m_deadzone) dvalueRYAxis = 0;

    // enforce maximums on joysticks
    if (Math.abs(dvalueLYAxis) > m_maxPower) {
      if (dvalueLYAxis > 0) dvalueLYAxis = m_maxPower; else dvalueLYAxis = -m_maxPower;
    }
    if (Math.abs(dvalueRYAxis) > m_maxPower) {
      if (dvalueRYAxis > 0) dvalueRYAxis = m_maxPower; else dvalueRYAxis = -m_maxPower;
    }

    if (false) {

    //System.out.println("Drive L: " + df3.format(dvalueLYAxis) + 
    //                    " R: " + df3.format(dvalueRYAxis));
    }

    if (!m_bTargeting) {
      this.JustDrive(dvalueLYAxis, dvalueRYAxis);
    } else {
      this.TargetDrive(dvalueLYAxis, dvalueRYAxis);
    }

   }

  private void TargetDrive(double l, double r) {
    double dDirection = m_visionTable.getEntry("TargetAngle").getDouble(0);
    double dFrame = m_visionTable.getEntry("FrameCounter").getDouble(0);

    if (true) {
    //System.out.println("Frame: " +df3.format(dFrame) +
   //                   " TargetAngle: " + df3.format(dDirection));
    }

    if (l == 0 && r == 0) {
      this.TargetDriveSpin(dDirection, dFrame);
      return;
    }

    // target while driving
    double targetAngle = dDirection;
    if (targetAngle > 0) {
      // turn right
      this.JustDrive(l,0);
    } else if (targetAngle < 0) {
      // turn left
      this.JustDrive(0,r);
    } else {
      // stop turning
      this.JustDrive(0, 0);
    }

  }

  private void TargetDriveSpin(double targetAngle, double frame) {
    if (targetAngle > 5) {
      // turn right
      this.JustDrive(1,-1);
    } else if (targetAngle < -5) {
      // turn left
      this.JustDrive(-1,1);
    } else {
      // stop turning
      this.JustDrive(0, 0);
    }
  }

  // sets the drive power for the left & right motors
  // corrects for the orientation of the mount
  private void JustDrive(double l, double r) {
    m_leftMotor.set(ControlMode.PercentOutput, l);
    m_rightMotor.set(ControlMode.PercentOutput, -r);  
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
