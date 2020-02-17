/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  private CANSparkMax m_LeftWinchMotor = null;
  private CANSparkMax m_RightWinchMotor = null;
  private CANSparkMax m_LeftLiftMotor = null;
  private CANSparkMax m_RightLiftMotor = null;
  private CANEncoder m_LLM_Encoder = null;
  private CANEncoder m_LRM_Encoder = null;
  private CANPIDController m_LLM_pidController = null;
  private CANPIDController m_LRM_pidController = null;

  private double m_LLM_pid_kP = 0; 
  private double m_LLM_pid_kI = 0.0;
  private double m_LLM_pid_kD = 0; 
  private double m_LLM_pid_kIz = 0; 
  private double m_LLM_pid_kFF = 0; 
  private double m_LLM_pid_kMaxOutput = 1.0; 
  private double m_LLM_pid_kMinOutput = -1.0;

  private double m_LRM_pid_kP = 0; 
  private double m_LRM_pid_kI = 0.0;
  private double m_LRM_pid_kD = 0; 
  private double m_LRM_pid_kIz = 0; 
  private double m_LRM_pid_kFF = 0; 
  private double m_LRM_pid_kMaxOutput = 1.0; 
  private double m_LRM_pid_kMinOutput = -1.0;

  private double m_liftExtensionTarget = 100; // the target extention for the lift motors
  private double m_WinchSpeed = 0.1;

  public Climber() {
    m_LeftWinchMotor = new CANSparkMax(33, MotorType.kBrushless);
    m_RightWinchMotor = new CANSparkMax(34, MotorType.kBrushless);
    m_LeftLiftMotor = new CANSparkMax(35, MotorType.kBrushless);
    m_RightLiftMotor = new CANSparkMax(36, MotorType.kBrushless);

    m_LLM_Encoder = m_LeftLiftMotor.getEncoder();
    m_LLM_Encoder.setPosition(0);
    m_LRM_Encoder = m_RightLiftMotor.getEncoder();
    m_LRM_Encoder.setPosition(0);


    m_LLM_pidController = m_LeftLiftMotor.getPIDController();
    m_LRM_pidController = m_RightLiftMotor.getPIDController();
    

        // Left Lift PID coefficients
        m_LLM_pid_kP = 0.7; 
        m_LLM_pid_kI = 0.0;
        m_LLM_pid_kD = 0; 
        m_LLM_pid_kIz = 0; 
        m_LLM_pid_kFF = 0; 
        m_LLM_pid_kMaxOutput = 1.0; 
        m_LLM_pid_kMinOutput = -1.0;
    
        // Set Left Lift PID coefficients
        m_LLM_pidController.setP(m_LLM_pid_kP);
        m_LLM_pidController.setI(m_LLM_pid_kI);
        m_LLM_pidController.setD(m_LLM_pid_kD);
        m_LLM_pidController.setIZone(m_LLM_pid_kIz);
        m_LLM_pidController.setFF(m_LLM_pid_kFF);
        m_LLM_pidController.setOutputRange(m_LLM_pid_kMinOutput, m_LLM_pid_kMaxOutput);
        
        // Right Lift PID coefficients
        m_LRM_pid_kP = 0.7; 
        m_LRM_pid_kI = 0.0;
        m_LRM_pid_kD = 0; 
        m_LRM_pid_kIz = 0; 
        m_LRM_pid_kFF = 0; 
        m_LRM_pid_kMaxOutput = 1.0; 
        m_LRM_pid_kMinOutput = -1.0;
    
        // set Right Lift PID coefficients
        m_LRM_pidController.setP(m_LRM_pid_kP);
        m_LRM_pidController.setI(m_LRM_pid_kI);
        m_LRM_pidController.setD(m_LRM_pid_kD);
        m_LRM_pidController.setIZone(m_LRM_pid_kIz);
        m_LRM_pidController.setFF(m_LRM_pid_kFF);
        m_LRM_pidController.setOutputRange(m_LRM_pid_kMinOutput, m_LRM_pid_kMaxOutput);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double pLLM = SmartDashboard.getNumber("Hood/Left Lift Motor/P Gain", 0);
    double iLLM = SmartDashboard.getNumber("Hood/Left Lift Motor/I Gain", 0);
    double dLLM = SmartDashboard.getNumber("Hood/Left Lift Motor/D Gain", 0);
    double izLLM = SmartDashboard.getNumber("Hood/Left Lift Motor/I Zone", 0);
    double ffLLM = SmartDashboard.getNumber("Hood/Left Lift Motor/Feed Forward", 0);
    double maxLLM = SmartDashboard.getNumber("Hood/Left Lift Motor/Max Output", 0);
    double minLLM = SmartDashboard.getNumber("Hood/Left Lift Motor/Min Output", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((pLLM !=  m_LLM_pid_kP)) { m_LLM_pidController.setP(pLLM);  m_LLM_pid_kP = pLLM; }
    if((iLLM !=  m_LLM_pid_kI)) { m_LLM_pidController.setI(iLLM);  m_LLM_pid_kI = iLLM; }
    if((dLLM !=  m_LLM_pid_kD)) { m_LLM_pidController.setD(dLLM);  m_LLM_pid_kD = dLLM; }
    if((izLLM !=  m_LLM_pid_kIz)) { m_LLM_pidController.setIZone(izLLM);  m_LLM_pid_kIz = izLLM; }
    if((ffLLM !=  m_LLM_pid_kFF)) { m_LLM_pidController.setFF(ffLLM);  m_LLM_pid_kFF = ffLLM; }
    if((maxLLM !=  m_LLM_pid_kMaxOutput) || (minLLM !=  m_LLM_pid_kMinOutput)) { 
      m_LLM_pidController.setOutputRange(minLLM, maxLLM); 
      m_LLM_pid_kMinOutput = minLLM;  m_LLM_pid_kMaxOutput = maxLLM; 
    }

    double pLRM = SmartDashboard.getNumber("Hood/Right Lift Motor/P Gain", 0);
    double iLRM = SmartDashboard.getNumber("Hood/Right Lift Motor/I Gain", 0);
    double dLRM = SmartDashboard.getNumber("Hood/Right Lift Motor/D Gain", 0);
    double izLRM = SmartDashboard.getNumber("Hood/Right Lift Motor/I Zone", 0);
    double ffLRM = SmartDashboard.getNumber("Hood/Right Lift Motor/Feed Forward", 0);
    double maxLRM = SmartDashboard.getNumber("Hood/Right Lift Motor/Max Output", 0);
    double minLRM = SmartDashboard.getNumber("Hood/Right Lift Motor/Min Output", 0);


    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((pLRM !=  m_LRM_pid_kP)) { m_LRM_pidController.setP(pLRM);  m_LRM_pid_kP = pLRM; }
    if((iLRM !=  m_LRM_pid_kI)) { m_LRM_pidController.setI(iLRM);  m_LRM_pid_kI = iLRM; }
    if((dLRM !=  m_LRM_pid_kD)) { m_LRM_pidController.setD(dLRM);  m_LRM_pid_kD = dLRM; }
    if((izLRM !=  m_LRM_pid_kIz)) { m_LRM_pidController.setIZone(izLRM);  m_LRM_pid_kIz = izLRM; }
    if((ffLRM !=  m_LRM_pid_kFF)) { m_LRM_pidController.setFF(ffLRM);  m_LRM_pid_kFF = ffLRM; }
    if((maxLRM !=  m_LRM_pid_kMaxOutput) || (minLRM !=  m_LRM_pid_kMinOutput)) { 
      m_LRM_pidController.setOutputRange(minLRM, maxLRM); 
      m_LRM_pid_kMinOutput = minLRM;  m_LRM_pid_kMaxOutput = maxLRM; 
    }
  }

  // Extends the hook lifts for climbing
  public void  extendLift() {
    m_LRM_pidController.setReference(m_liftExtensionTarget, ControlType.kPosition); //sets the motor to go to the lift target
    m_LLM_pidController.setReference(m_liftExtensionTarget, ControlType.kPosition); //sets the motor to go to the lift target

    return;
  }

  // Retracts both hook at the same time

  public void retractLift() {
    m_LRM_pidController.setReference(0, ControlType.kPosition); //sets the motor to go to the orginal position
    m_LLM_pidController.setReference(0, ControlType.kPosition); //sets the motor to go to the orginal position

    return;
  }

  public void releaseWinch()
  {
    m_LeftWinchMotor.set(m_WinchSpeed);
    m_RightWinchMotor.set(m_WinchSpeed);
  }

  public void stopReleaseWinch()
  {
    m_LeftWinchMotor.set(0);
    m_RightWinchMotor.set(0);
  }

  public void retractWinch()
  {
    m_LeftWinchMotor.set(-m_WinchSpeed);
    m_RightWinchMotor.set(-m_WinchSpeed);
  }

  public void stopRetractWinch()
  {
    m_LeftWinchMotor.set(0);
    m_RightWinchMotor.set(0);
  }

  // Retracts left hook
  // Returns false if not fully retracted
  // Returns true if fully retracted
  public boolean retractLeft() {
    return false;
  }

  // Retracts right hook
  // Returns false if not fully retracted
  // Returns true if fully retracted
  public boolean retractRight() {
    return false;
  }

}
