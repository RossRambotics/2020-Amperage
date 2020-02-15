/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Rev Spark Max classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// ultra sonic sensor classes
import edu.wpi.first.wpilibj.AnalogInput;

import frc.robot.commands.IndexerCheckForNewPowerCell;

public class Indexer extends SubsystemBase {
  private double m_TopMotorPower = 0.25;
  private double m_CompactPower = 0.10;
  private double m_dCompactRotations = 1.0;
  private CANSparkMax m_bottomMotor = null;
  private CANSparkMax m_topMotor = null;
  private CANEncoder m_encoderBottom = null;
  private CANPIDController m_pidControllerBottom = null;  
  private CANEncoder m_encoderTop = null;
  private CANPIDController m_pidControllerTop = null;
  private double m_pid_Kp = 0.2;

  // setup ultra sonic sensor
  public AnalogInput m_Sensor_PC_Intake = new AnalogInput(0);  // intake has presented powercell to indexer
  public AnalogInput m_Sensor_PC_Index0 = new AnalogInput(1);  // intake has presented powercell to indexer
  //public AnalogInput m_Sensor_PC_Capture = new AnalogInput(1); // ball as been captured in indexer


  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    // TODO fix the CAN id of the motors
    m_bottomMotor =  new CANSparkMax(3, MotorType.kBrushless);
    m_topMotor =  new CANSparkMax(4, MotorType.kBrushless);
    m_bottomMotor.restoreFactoryDefaults();
    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setInverted(true);
    m_bottomMotor.setInverted(true);

    m_encoderBottom = m_bottomMotor.getEncoder();
    m_encoderBottom.setPosition(0);
    m_pidControllerBottom = m_bottomMotor.getPIDController();
    m_pidControllerBottom.setP(m_pid_Kp);

    m_encoderTop = m_topMotor.getEncoder();
    m_encoderTop.setPosition(0);
    m_pidControllerTop = m_topMotor.getPIDController();
    m_pidControllerTop.setP(m_pid_Kp);
    
    // display variables on SmartDashboard
    SmartDashboard.putBoolean("Indexer/Sensor_0_Intake", false);
    SmartDashboard.putBoolean("Indexer/Sensor_1_Index0", false);
    SmartDashboard.putNumber("Indexer/Sensor_0_Intake Raw", m_Sensor_PC_Intake.getValue());
    SmartDashboard.putNumber("Indexer/Top Motor Power", m_TopMotorPower);
    SmartDashboard.putNumber("Indexer/Compact Power", m_CompactPower);
    SmartDashboard.putNumber("Indexer/Compact Rotations", m_dCompactRotations);
    SmartDashboard.putNumber("Indexer/Compact Encoder Top", m_encoderTop.getPosition());
    SmartDashboard.putNumber("Indexer/Compact Encoder Bottom", m_encoderBottom.getPosition());
    SmartDashboard.putNumber("Indexer/Compact Kp", m_pid_Kp);

    this.setDefaultCommand(new IndexerCheckForNewPowerCell(this));

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    

     // display variables on SmartDashboard
     boolean b = (m_Sensor_PC_Intake.getValue() > 10) ? false : true;
     SmartDashboard.putBoolean("Indexer/Sensor_0_Intake", b);
     SmartDashboard.putNumber("Indexer/Sensor_0_Intake Raw", m_Sensor_PC_Intake.getValue());
     SmartDashboard.putNumber("Indexer/Sensor_1_Index0", m_Sensor_PC_Index0.getValue());
     SmartDashboard.putNumber("Indexer/Compact Encoder Top", m_encoderTop.getPosition());
     SmartDashboard.putNumber("Indexer/Compact Encoder Bottom", m_encoderBottom.getPosition());
     double Kp = SmartDashboard.getNumber("Indexer/Compact Kp", 0);
     if (Kp != m_pid_Kp) {
        m_pid_Kp = Kp;
        m_pidControllerTop.setP(m_pid_Kp);
        m_pidControllerBottom.setP(m_pid_Kp);
     }

     m_TopMotorPower = SmartDashboard.getNumber("Indexer/Top Motor Power", 0);
     m_CompactPower = SmartDashboard.getNumber("Indexer/Compact Power", 0);
     m_dCompactRotations = SmartDashboard.getNumber("Indexer/Compact Rotations", 0);
  }

  // returns true when the index should capture the powercell
  // returns false when no powercell is present
  public boolean SenseIntakePC() {
    boolean b;

    if (m_Sensor_PC_Intake.getValue() > 10) {
      b = false;
    } else {
      b = true;
    }
    return b;
  }  
  
  // returns true when the index advance new powercell
  // returns false when indexer/intake is clear
  public boolean SenseIndex0() {
    boolean b;

    if (m_Sensor_PC_Index0.getValue() > 10) {
      b = false;
    } else {
      b = true;
    }
    return b;
  }



  // moves the balls to the next sensor
  // returns false if balls are still moving/inbetween
  // returns true if the balls finished moving
  public boolean advance() {
    // set the indexer motors to run
    m_bottomMotor.set(m_TopMotorPower);
    m_topMotor.set(m_TopMotorPower);
    return false;
  }

  // runs until all of the balls are shot out
  // returns false if not all of the balls are out
  // returns true is all of the balls are out 
  public boolean shoot() {
    // set the indexer motors to run
    //m_bottomMotor.set(0.75);
    //m_topMotor.set(0.75);

    // TODO detect whether there are balls remaining
    return false;
  }

  public void stop() {
    // stops the indexer motors
    m_topMotor.set(0);
    m_bottomMotor.set(0);
  }

  // makes the balls go backward out of the intake
  // returns false if the balls did not move back
  // returns true of the balls did move backwards 
  public boolean clear() {
    m_bottomMotor.set(-0.25);
    m_topMotor.set(-0.25);
  return false;
  }


  // move the balls using the top and bottom motors
  public void compact() {
    // reset encoders and advance distance
    m_encoderTop.setPosition(0);
    m_encoderBottom.setPosition(0);
    m_pidControllerBottom.setReference(m_dCompactRotations, ControlType.kPosition);
    m_pidControllerTop.setReference(m_dCompactRotations, ControlType.kPosition);
  }
}
