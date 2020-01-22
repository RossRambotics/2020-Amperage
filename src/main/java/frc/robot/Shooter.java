/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    //shoots the balls 
    //returns false if empty
    //returns true if balls remain
    public boolean shoot() {
      return false;
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
}
