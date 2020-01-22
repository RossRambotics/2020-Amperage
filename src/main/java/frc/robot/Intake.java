/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
    //retracts the intake 
    //returns false if extended 
    //returns true if fully retracted 
    public boolean retract() {
      return false;
    }

    //extends the intake 
    //returns false if retracted
    //returns true if fully extended
    public boolean extend() {
      return false;
    }
    //spins the intake to capture a ball 
    //returns false if a ball is not being captured 
    //returns true if a ball is being captured 
    public boolean capture() {
      return false;
    }
    //reverse spins the intake in case of jam
    //return true if cleared
    //return false if jam is still detected
    public boolean clear () {
      return false;
    }
    //shoots the ball 
    //returns false if intake is empty
    //returns true if has ball in intake
    public boolean shoot() {
      return false;
    
  }
}
