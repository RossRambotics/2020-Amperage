/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;

public class TargetingRelay extends SubsystemBase {
  boolean m_bIsOn = false;  // true if the relay is on
  private final DigitalOutput m_LEDrelay = new DigitalOutput(0); // digital output tied to the realy


  /**
   * Creates a new TargetingRelay.
   */
  public TargetingRelay() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_LEDrelay.set(m_bIsOn);
  }
}
