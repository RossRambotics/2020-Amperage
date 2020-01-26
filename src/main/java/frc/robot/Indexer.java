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

public class Indexer extends SubsystemBase {
  private CANSparkMax m_bottomMotor = null;
  private CANSparkMax m_topMotor = null;


  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    // TODO fix the CAN id of the motors
    m_bottomMotor =  new CANSparkMax(11, MotorType.kBrushless);
    m_topMotor =  new CANSparkMax(11, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // moves the balls to the next sensor
  // returns false if balls are still moving/inbetween
  // returns true if the balls finished moving
  public boolean advance() {
    return false;
  }

  // runs until all of the balls are shot out
  // returns false if not all of the balls are out
  // returns true is all of the balls are out 
  public boolean shoot() {
    // set the indexer motors to run
    m_bottomMotor.set(0.25);
    m_topMotor.set(0.25);

    // TODO detect whether there are balls remaining
    return false;
  }

  public void stop() {
    // stops the indexer motors
    m_bottomMotor.set(0);
    m_bottomMotor.set(0);
  }

  // makes the balls go backward out of the intake
  // returns false if the balls did not move back
  // returns true of the balls did move backwards 
    public boolean clear() {
    return false;
  }
}
