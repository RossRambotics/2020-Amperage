/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(15, 0, 1);// creates the solenoid on CAN id 15

  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
    //retracts the intake 
    public void retract() {
      intakeSolenoid.set(Value.kReverse);
    }

    //extends the intake 
    public void extend() {
      intakeSolenoid.set(Value.kForward);
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
