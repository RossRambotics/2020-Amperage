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
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import frc.robot.TheRobot;
import frc.robot.helper.ShooterLookUp;
import frc.robot.helper.ShooterValueSet;
import frc.robot.Robot;


public class Hood extends SubsystemBase {
  private CANSparkMax m_motorHood = null;
  private CANEncoder m_encoderHood = null;
  private CANPIDController m_pidController = null;

  private ShooterLookUp m_lookUpTable = null; // look up table for shooter values

  private boolean m_extended = false;

  private double m_position_hood = 0;
  private double m_position_target = 5000;

  private double m_pid_kP, m_pid_kI, m_pid_kD, m_pid_kIz, m_pid_kFF;
  private double m_pid_kMaxOutput, m_pid_kMinOutput, m_pid_maxRPM;

  // 55/18 --- ratio of the 2 sprockets
  // * 4   --- 4:1 gear box
  // / 360 --- degrees per rotation of the hood
  private static final double kRotationsPerDegree = ((55 / 18) * 4) / 360; 

  /**
   * Creates a new Shooter.
   */
  public Hood() {
    // TODO fix the CAN id of the motors
    // setup motor
    m_motorHood =  new CANSparkMax(55, MotorType.kBrushless);
    m_motorHood.restoreFactoryDefaults();

    m_encoderHood = m_motorHood.getEncoder();
    m_encoderHood.setPosition(0);
    m_pidController = m_motorHood.getPIDController();

    // PID coefficients
    m_pid_kP = 5e-5; 
    m_pid_kI = 1e-6;
    m_pid_kD = 0; 
    m_pid_kIz = 0; 
    m_pid_kFF = 0; 
    m_pid_kMaxOutput = 1; 
    m_pid_kMinOutput = -1;
    m_pid_maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(m_pid_kP);
    m_pidController.setI(m_pid_kI);
    m_pidController.setD(m_pid_kD);
    m_pidController.setIZone(m_pid_kIz);
    m_pidController.setFF(m_pid_kFF);
    m_pidController.setOutputRange(m_pid_kMinOutput, m_pid_kMaxOutput);

    m_lookUpTable = new ShooterLookUp();

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Hood P Gain", m_pid_kP);
    SmartDashboard.putNumber("Hood I Gain", m_pid_kI);
    SmartDashboard.putNumber("Hood D Gain", m_pid_kD);
    SmartDashboard.putNumber("Hood I Zone", m_pid_kIz);
    SmartDashboard.putNumber("Hood Feed Forward", m_pid_kFF);
    SmartDashboard.putNumber("Hood Max Output", m_pid_kMaxOutput);
    SmartDashboard.putNumber("Hood Min Output", m_pid_kMinOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Hood P Gain", 0);
    double i = SmartDashboard.getNumber("Hood I Gain", 0);
    double d = SmartDashboard.getNumber("Hood D Gain", 0);
    double iz = SmartDashboard.getNumber("Hood I Zone", 0);
    double ff = SmartDashboard.getNumber("Hood Feed Forward", 0);
    double max = SmartDashboard.getNumber("Hood Max Output", 0);
    double min = SmartDashboard.getNumber("Hood Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p !=  m_pid_kP)) { m_pidController.setP(p);  m_pid_kP = p; }
    if((i !=  m_pid_kI)) { m_pidController.setI(i);  m_pid_kI = i; }
    if((d !=  m_pid_kD)) { m_pidController.setD(d);  m_pid_kD = d; }
    if((iz !=  m_pid_kIz)) { m_pidController.setIZone(iz);  m_pid_kIz = iz; }
    if((ff !=  m_pid_kFF)) { m_pidController.setFF(ff);  m_pid_kFF = ff; }
    if((max !=  m_pid_kMaxOutput) || (min !=  m_pid_kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      m_pid_kMinOutput = min;  m_pid_kMaxOutput = max; 
    }

    if (m_extended == true) {
      ShooterValueSet m_values = m_lookUpTable.getCurrentValues(false);
      m_position_target = m_values.hoodAngle * Hood.kRotationsPerDegree;  
    } else {
      m_position_target = 0;
    }

    m_pidController.setReference(m_position_target, ControlType.kPosition);

    // Get the position of the hood
    m_position_hood = Math.abs(m_encoderHood.getPosition());

    // Output to dashboard
    SmartDashboard.putNumber("Hood Current Position", m_position_hood);
    SmartDashboard.putNumber("Hood Target Position", m_position_target);
  }

  //shoots the balls 
  public void shoot() {
    Robot r = TheRobot.getInstance();
 
    // get distance to target
    double d = r.m_powerPowerTargeter.getDistance();
    ShooterValueSet m_values = m_lookUpTable.getCurrentValues(false);

    // tell shooter to come up to target speed based on distance
  
    if (r.m_shooter.ready(m_values)) {
      // start the indexer
      //r.m_indexer.shoot();
    } else {
      // stop the indexer
      //r.m_indexer.stop();
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
      m_extended = false;
      m_position_target = 0;
      return false;
    }
    
     //extends the hood
    //returns false if not exteneded
    //returns true if exteneded
    public boolean extend() {
      m_extended = true;
      return false;
    
    }

    public void stop() {
      // stops the shooter motors
      m_motorHood.set(0);

      // also make sure the indexer stops
      Robot r = TheRobot.getInstance();
      //r.m_indexer.stop();
    }


  // returns true if the shooter is up-to-speed for the target distance
  // if distance is zero takes hood to default position
  // returns false if the hood is not at target position
  public boolean ready(ShooterValueSet m_Values) {
    m_position_target = m_Values.hoodAngle;  // default target speed
    double maxPower = 0.5;      // default maximum power

    // TODO lookup target speed based on distance

    // set the PID Controller to hit the position
    m_pidController.setReference(m_position_target, ControlType.kPosition);
    TheRobot.log("Shooter ready RPM_target:" + TheRobot.toString(m_position_target));

    // See if motor hood are within range tolerance
    double range = 2;
    if (Math.abs(m_position_target - m_position_hood) < range) {
      return true;
    } 
    
    return false;
  }
}
