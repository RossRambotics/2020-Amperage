/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Rev Spark Max classes
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// ultra sonic sensor classes
import edu.wpi.first.wpilibj.AnalogInput;

import frc.robot.commands.IndexerCheckForNewPowerCell;

public class Indexer extends SubsystemBase {
  private CANSparkMax m_bottomMotor = null;
  private CANSparkMax m_topMotor = null;

  // setup ultra sonic sensor
  public AnalogInput m_Sensor_PC_Intake = new AnalogInput(0);  // intake has presented powercell to indexer
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
    m_topMotor.setInverted(false);
    m_bottomMotor.setInverted(false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.setDefaultCommand(new IndexerCheckForNewPowerCell());
  }

  // returns true when the index should capture the powercell
  // returns false when no powercell is present
  public boolean SenseIntakePC() {
    boolean b;

    if (m_Sensor_PC_Intake.getValue() != 0) {
      b = true;
    } else {
      b = false;
    }
    return b;
  }



  // moves the balls to the next sensor
  // returns false if balls are still moving/inbetween
  // returns true if the balls finished moving
  public boolean advance() {
    // set the indexer motors to run
    //m_bottomMotor.set(0.45);
    m_topMotor.set(0.75);
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
    m_bottomMotor.set(-0.50);
    m_topMotor.set(-0.50);
  return false;
  }
}
