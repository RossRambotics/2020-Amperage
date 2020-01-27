/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TheRobot;

// network tables
import edu.wpi.first.networktables.NetworkTable;


public class PowerPortTargeter extends SubsystemBase {
  /**
   * Creates a new PowerPortTargeter.
   */
  public PowerPortTargeter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // returns the distance to the Power Port
  // returns zero if unknown
  public double getDistance() {
    // get the distance to the target from the network tables
    NetworkTable n = TheRobot.getInstance().m_drive.GetNetworkTable();
    double v = n.getEntry("TargetDistance").getDouble(0);
    TheRobot.log("TargetDistance: " + TheRobot.toString(v));

    return v;
  }
}
