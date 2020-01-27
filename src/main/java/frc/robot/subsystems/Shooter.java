/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Rev Spark Max classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.TheRobot;
import frc.robot.Robot;


public class Shooter extends SubsystemBase {
  private CANSparkMax m_motor1 = null;
  private CANSparkMax m_motor2 = null;
  private CANEncoder m_encoder1 = null;

  private double m_RPM_shooter = 0;
  private double m_RPM_target = 5000;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // TODO fix the CAN id of the motors
    m_motor1 =  new CANSparkMax(30, MotorType.kBrushless);
    m_motor2 =  new CANSparkMax(30, MotorType.kBrushless);
    m_motor2.follow(m_motor1, true);

    m_encoder1 = m_motor1.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get the RPM of the motors
    m_RPM_shooter = Math.abs(m_encoder1.getVelocity());

    // Output to dashboard
    SmartDashboard.putNumber("Shooter Current RPM", m_RPM_shooter);
    SmartDashboard.putNumber("Shooter Target RPM", m_RPM_target);
  }

  //shoots the balls 
  public void shoot() {
    Robot r = TheRobot.getInstance();
 
    // get distance to target
    double d = r.m_powerPowerTargeter.getDistance();

    // tell shooter to come up to target speed based on distance
  
    if (r.m_shooter.ready(d)) {
      // start the indexer
      r.m_indexer.shoot();
    } else {
      // stop the indexer
      r.m_indexer.stop();
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
    m_RPM_target = 5000;  // default target speed
    double maxPower = 0.5;      // default maximum power

    // TODO lookup target speed based on distance

    // Get Motor RPMs
    // TODO check default target speed!
    double actualSpeed = 5000;
    actualSpeed = m_RPM_shooter;

    // See if motor RPM are within range tolerance
    double range = 200;
    if (Math.abs(m_RPM_target - actualSpeed) < range) {
      return true;
    } else {
      // TODO adjust speed
      m_motor1.set(maxPower);
    }

    
    return false;
  }
}
