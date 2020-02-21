/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ledColor;

public class ledController extends SubsystemBase {
  /**
   * Creates a new ledController.
   */

  private Spark m_ledController = new Spark(1);


  public ledController() {

  
  }

  @Override
  public void periodic() {
    
  }

  public void setColor(ledColor color) {

    switch(color) {
      case kIndexerFull:
        m_ledController.set(0.97);
        break;

      case kTargetFound:
        m_ledController.set(0.79);
        break;
    }

  }

}
