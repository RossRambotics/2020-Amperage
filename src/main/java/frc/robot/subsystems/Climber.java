/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Extends the hooks for climbing
  // Returns false if not fully extended
  // Returns true if fully extended
  public boolean extend() {
    return false;
  }

  // Retracts both hook at the same time
  // Returns false if not fully retracted
  // Returns true if fully retracted
  public boolean retract() {
    return false;
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
