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


public class Shooter extends SubsystemBase {
  private CANSparkMax m_motor1 = null;
  private CANSparkMax m_motor2 = null;
  private CANEncoder m_encoder1 = null;
  private CANPIDController m_pidController = null;

  private ShooterLookUp m_lookUpTable = null; // look up table for shooter values

  private double m_RPM_shooter = 0;
  private double m_RPM_target = 5000;

  private double m_pid_kP, m_pid_kI, m_pid_kD, m_pid_kIz, m_pid_kFF;
  private double m_pid_kMaxOutput, m_pid_kMinOutput, m_pid_maxRPM;
  private boolean m_bTuning = false;
  private double m_dTuningRPM = 0;
  

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // TODO fix the CAN id of the motors
    // setup motors
    m_motor1 =  new CANSparkMax(2, MotorType.kBrushless);
    m_motor2 =  new CANSparkMax(1, MotorType.kBrushless);
    m_motor2.follow(m_motor1, true);
    m_motor1.setSmartCurrentLimit(20, 5500);  // limit motor to 20A & 5500 RPM
    m_motor2.setSmartCurrentLimit(20, 5500);  // limit motor to 20A & 5500 RPM

    m_encoder1 = m_motor1.getEncoder();
    m_pidController = m_motor1.getPIDController();

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
    SmartDashboard.putNumber("Shooter/P Gain", m_pid_kP);
    SmartDashboard.putNumber("Shooter/I Gain", m_pid_kI);
    SmartDashboard.putNumber("Shooter/D Gain", m_pid_kD);
    SmartDashboard.putNumber("Shooter/I Zone", m_pid_kIz);
    SmartDashboard.putNumber("Shooter/Feed Forward", m_pid_kFF);
    SmartDashboard.putNumber("Shooter/Max Output", m_pid_kMaxOutput);
    SmartDashboard.putNumber("Shooter/Min Output", m_pid_kMinOutput);
    SmartDashboard.putBoolean("Shooter/Tuning Mode", m_bTuning);
    SmartDashboard.putNumber("Shooter/Tuning RPM", m_dTuningRPM);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Shooter/P Gain", 0);
    double i = SmartDashboard.getNumber("Shooter/I Gain", 0);
    double d = SmartDashboard.getNumber("Shooter/D Gain", 0);
    double iz = SmartDashboard.getNumber("Shooter/I Zone", 0);
    double ff = SmartDashboard.getNumber("Shooter/Feed Forward", 0);
    double max = SmartDashboard.getNumber("Shooter/Max Output", 0);
    double min = SmartDashboard.getNumber("Shooter/Min Output", 0);
    m_bTuning = SmartDashboard.getBoolean("Shooter/Tuning Mode", m_bTuning);
    m_dTuningRPM = SmartDashboard.getNumber("Shooter/Tuning RPM", m_dTuningRPM);



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

    // Get the RPM of the motors
    m_RPM_shooter = Math.abs(m_encoder1.getVelocity());

    // Output to dashboard
    SmartDashboard.putNumber("Shooter/Current RPM", m_RPM_shooter);
    SmartDashboard.putNumber("Shooter/Target RPM", m_RPM_target);
  }

  //shoots the balls 
  public void shoot() {
    Robot r = TheRobot.getInstance();
 
    // get distance to target
    ShooterValueSet m_values = m_lookUpTable.getCurrentValues(true);
    System.out.println(m_values);
    // tell shooter to come up to target speed based on distance  
    if (r.m_shooter.ready(m_values)) {
      // start the indexer
      r.m_indexer.shoot();
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
    
    public void stop() {
      // stops the shooter motors
      m_motor1.set(0);

      // also make sure the indexer stops
      Robot r = TheRobot.getInstance();
      //r.m_indexer.stop();

      // retract the hood
      r.m_hood.retract();
    }


  // returns true if the shooter is up-to-speed for the target distance
  // if distance is zero takes shooter to default speed
  // returns false if the shooter is not at target speed
  public boolean ready(ShooterValueSet m_Values) {
    // set the target RPM
    //m_RPM_target = m_Values.shooterRPM;
    m_RPM_target = m_dTuningRPM;

    // set the PID Controller to hit the RPM
    m_pidController.setReference(m_RPM_target, ControlType.kVelocity);
    TheRobot.log("Shooter ready RPM_target:" + TheRobot.toString(m_RPM_target));

    // See if motor RPM are within range tolerance
    double range = 100;
    if (Math.abs(m_RPM_target - m_RPM_shooter) < range) {
      return true;
    } 
    
    return false;
  }
}
