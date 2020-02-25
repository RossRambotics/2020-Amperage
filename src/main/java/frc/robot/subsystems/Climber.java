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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TheRobot;
import frc.robot.eRobotSide;
import frc.robot.commands.ExtendClimbLift;

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
  private CANEncoder m_WLM_Encoder = null;
  private CANEncoder m_WRM_Encoder = null;
  private CANPIDController m_LLM_pidController = null;
  private CANPIDController m_LRM_pidController = null;

  private double m_liftWinchConstant = .75; // lift roations relative to winch roations

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
  private double m_WinchSpeed = 0.5;
  private double m_LiftSpeed = 1.0;
  private Joystick m_OperatorStick;
  private double m_deadzone = 0.05;


  public Climber() {
    m_LeftWinchMotor = new CANSparkMax(7, MotorType.kBrushless);
    m_RightWinchMotor = new CANSparkMax(8, MotorType.kBrushless);
    m_LeftLiftMotor = new CANSparkMax(9, MotorType.kBrushless);
    m_RightLiftMotor = new CANSparkMax(10, MotorType.kBrushless);

    double dRampRate = 0;
    m_LeftWinchMotor.setOpenLoopRampRate(dRampRate);
    m_RightWinchMotor.setOpenLoopRampRate(dRampRate);
    m_RightWinchMotor.setInverted(true);
    m_LeftLiftMotor.setOpenLoopRampRate(dRampRate);
    m_RightLiftMotor.setOpenLoopRampRate(dRampRate);


    m_LLM_Encoder = m_LeftLiftMotor.getEncoder();
    m_LLM_Encoder.setPosition(0);
    m_LRM_Encoder = m_RightLiftMotor.getEncoder();
    m_LRM_Encoder.setPosition(0);

    m_WLM_Encoder = m_LeftWinchMotor.getEncoder();
    m_WLM_Encoder.setPosition(0);
    m_WRM_Encoder = m_RightWinchMotor.getEncoder();
    m_WRM_Encoder.setPosition(0);

    SmartDashboard.putNumber("Climber/Left Motor Rotations", m_LLM_Encoder.getPosition());
    SmartDashboard.putNumber("Climber/Right Motor Rotations", m_LRM_Encoder.getPosition());

    m_OperatorStick =  new Joystick(1); // Operator joystick

    m_LeftLiftMotor.setIdleMode(IdleMode.kBrake);
    m_RightLiftMotor.setIdleMode(IdleMode.kBrake);
    m_LeftWinchMotor.setIdleMode(IdleMode.kCoast);
    m_RightWinchMotor.setIdleMode(IdleMode.kCoast);
/*
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
    */

    this.setDefaultCommand(new ExtendClimbLift(this));
  }

  @Override
  public void periodic() {

    //TheRobot.log("In Climber periodic.");

    //check if it says Left!
    double dvalueLXAxis = m_OperatorStick.getRawAxis(0);
    double dvalueLYAxis = -m_OperatorStick.getRawAxis(1); // joystick Y axis is inverted
    double dvalueRXAxis = m_OperatorStick.getRawAxis(4);
    double dvalueRYAxis = -m_OperatorStick.getRawAxis(5); // joystick Y axis is inverted

    // enforce deadzone on joysticks
    if (Math.abs(dvalueLXAxis) < m_deadzone) dvalueLXAxis = 0;
    if (Math.abs(dvalueLYAxis) < m_deadzone) dvalueLYAxis = 0;
    if (Math.abs(dvalueRXAxis) < m_deadzone) dvalueRXAxis = 0;
    if (Math.abs(dvalueRYAxis) < m_deadzone) dvalueRYAxis = 0;

    // TODO --- Get rid of these???
    SmartDashboard.getEntry("LWEPosition").setDouble(m_WLM_Encoder.getPosition());
    SmartDashboard.getEntry("LLEPosition").setDouble(m_LLM_Encoder.getPosition());
    SmartDashboard.getEntry("m_liftWinchConstant").setDouble(m_liftWinchConstant);

    SmartDashboard.putNumber("Climber/Left Motor Rotations", m_LLM_Encoder.getPosition());
    SmartDashboard.putNumber("Climber/Right Motor Rotations", m_LRM_Encoder.getPosition());

    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
  /*  double pLLM = SmartDashboard.getNumber("Hood/Left Lift Motor/P Gain", 0);
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
    */
  }

  // Extends the hook lifts for climbing
  public void  extendLift(eRobotSide w, double p) {
    // TODO put a maximum 

    //m_LRM_pidController.setReference(m_liftExtensionTarget, ControlType.kPosition); //sets the motor to go to the lift target
    //m_LLM_pidController.setReference(m_liftExtensionTarget, ControlType.kPosition); //sets the motor to go to the lift target

    if (w == eRobotSide.LEFT) {
      m_LeftLiftMotor.set(p);
    } else {
      m_RightLiftMotor.set(p);
    }

    return;
  }

  // Retracts both hook at the same time

  public void releaseLift(eRobotSide w) {
    // TODO stop at zero

    if (w == eRobotSide.LEFT) {
      m_LeftLiftMotor.set(-m_LiftSpeed);
    } else {
      m_RightLiftMotor.set(-m_LiftSpeed);
    }

    //m_LRM_pidController.setReference(0, ControlType.kPosition); //sets the motor to go to the orginal position
    //m_LLM_pidController.setReference(0, ControlType.kPosition); //sets the motor to go to the orginal position

    return;
  }

  public void stopLift(eRobotSide w)
  {
    if (w == eRobotSide.LEFT) {
      m_LeftLiftMotor.set(0.0);
    } else {
      m_RightLiftMotor.set(0.0);
    }
  }
  public void releaseWinch(eRobotSide w)
  {
    if (w == eRobotSide.LEFT) {
      m_LeftWinchMotor.set(-m_WinchSpeed);
    } else {
      m_RightWinchMotor.set(-m_WinchSpeed);
    }
  }

  public void stopWinch(eRobotSide w)
  {
    if (w == eRobotSide.LEFT) {
      m_LeftWinchMotor.set(0.0);
    } else {
      m_RightWinchMotor.set(0.0);
    }
  }

  public void retractWinch(eRobotSide w)
  {
    if (w == eRobotSide.LEFT) {
      m_LeftWinchMotor.set(-m_WinchSpeed);
    } else {
      m_RightWinchMotor.set(m_WinchSpeed);
    }
  }

  public void syncExtend(eRobotSide w, double power)
  {
    double powerReductionK = .2;
    double maxPower = m_WinchSpeed * power;

    if (w == eRobotSide.LEFT) {
      if(Math.abs(maxPower) < 0.01){
        m_LeftWinchMotor.set(0);
        m_LeftLiftMotor.set(0);
        return;
      }

      double liftOffsetRotations = m_liftWinchConstant * (-m_WLM_Encoder.getPosition()) - m_LLM_Encoder.getPosition(); 
      // positive winch is ahead
      // negative encoder is ahead
      m_liftWinchConstant = SmartDashboard.getEntry("m_liftWinchConstant").getDouble(m_liftWinchConstant);
      double powerReduction = maxPower * Math.abs(liftOffsetRotations) * powerReductionK;
      if(powerReduction > maxPower){powerReduction = maxPower;}
     
      if(liftOffsetRotations > 0){
        SmartDashboard.getEntry("LeftLiftPower").setDouble(maxPower);
        SmartDashboard.getEntry("LeftWinchPower").setDouble(-(maxPower - powerReduction));
        m_LeftWinchMotor.set(-(maxPower - powerReduction));
        m_LeftLiftMotor.set(maxPower);
      }
      else{
        SmartDashboard.getEntry("LeftLiftPower").setDouble(maxPower - powerReduction);
        SmartDashboard.getEntry("LeftWinchPower").setDouble(-maxPower);
        m_LeftWinchMotor.set(-(maxPower));
        m_LeftLiftMotor.set(maxPower - powerReduction);
      }
    } else {
      if(Math.abs(maxPower) < 0.01){
        m_RightWinchMotor.set(0);
        m_RightLiftMotor.set(0);
        return;
      }

      double liftOffsetRotations = m_liftWinchConstant * (-m_WRM_Encoder.getPosition()) - m_LRM_Encoder.getPosition(); 
      // positive winch is ahead
      // negative encoder is ahead
      double powerReduction = maxPower * Math.abs(liftOffsetRotations) * powerReductionK;
      if(powerReduction > maxPower){powerReduction = maxPower;}
     
      if(liftOffsetRotations > 0){
        m_RightWinchMotor.set(-(maxPower - powerReduction));
        m_RightLiftMotor.set(maxPower);
      }
      else{
        m_RightWinchMotor.set(-(maxPower));
        m_RightLiftMotor.set(maxPower - powerReduction);
      }
    }
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
