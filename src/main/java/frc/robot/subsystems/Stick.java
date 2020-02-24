/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stick extends SubsystemBase {

  private Joystick m_joystick;

  /**
   * Creates a new Stick.
   * 
   * @param joystick
   */
  public Stick(Joystick joystick) {
    m_joystick = joystick;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRumble(RumbleType rumbleType) {
    m_joystick.setRumble(rumbleType, 0.5);
  }

  public void clearRumble(RumbleType rumbleType) {
    m_joystick.setRumble(rumbleType, 0.0);

  }
}
