/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Rev Spark Max classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.Shoot;


public class Shooter extends SubsystemBase {
  private CANSparkMax m_motor1 = null;
  private CANSparkMax m_motor2 = null;

  private Shoot m_shootCMD = null;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // TODO fix the CAN id of the motors
    m_motor1 =  new CANSparkMax(11, MotorType.kBrushless);
    m_motor2 =  new CANSparkMax(11, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 
 
  }

  //shoots the balls 
  public void shoot() {

    // If there is a prior shooting command ignore because we are already shooting!
    if (m_shootCMD != null) {
      // Check to see if we are done shooting
      if (m_shootCMD.isFinished()) {
        m_shootCMD = null;
      }
      // continue current shoot request
      return;
    } else {
      // create new shoot command
      m_shootCMD = new Shoot();
      TheRobot.getInstance().m_CMDScheduler.schedule(m_shootCMD);
    }
  }

    //runs the shooter backwards in case of a jam
    //returns true if no jam detected
    //returns false if jam detected
    public boolean clear() {
      return false;
    }

    //retracts the hood
    //returns false if not retracted
    //returns true if retracted
    public boolean retract() {
      return false;
    }
    
     //extends the hood
    //returns false if not exteneded
    //returns true if exteneded
    public boolean extend() {
      return false;
    
    }

    public void stop() {
      // stops the shooter motors
      m_motor1.set(0);
      m_motor2.set(0);

      // also make sure the indexer stops
      Robot r = TheRobot.getInstance();
      r.m_indexer.stop();
    }


  // returns true if the shooter is up-to-speed for the target distance
  // if distance is zero takes shooter to default speed
  // returns false if the shooter is not at target speed
  public boolean ready(double distance) {
    double targetSpeed = 5000;  // default target speed
    double maxPower = 0.5;      // default maximum power

    // TODO lookup target speed based on distance

    // TODO get current speed from encoders
    double actualSpeed = 5000;

    if (targetSpeed == actualSpeed) {
      return true;
    } else {
      // TODO adjust speed
      m_motor1.set(maxPower);
      m_motor2.set(-maxPower);
    }

    
    return false;
  }
}
