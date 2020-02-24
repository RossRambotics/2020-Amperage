/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ledColor;

public class PowerCellTargeter extends SubsystemBase {
    private boolean m_isInverted = false;
    private NetworkTableInstance m_ntInst = null;
    private NetworkTable m_ntTble = null;

  public PowerCellTargeter() {
    m_ntInst = NetworkTableInstance.getDefault();
    m_ntTble = m_ntInst.getTable("PCContourTable"); // gets the networktable where the powercell information is stored
  }

  @Override
  public void periodic() {
    
  }

  public double getPowerCellAngle(){
    if(!m_isInverted){return m_ntTble.getEntry("PowerCellAngle").getDouble(0);}
    return -m_ntTble.getEntry("PowerCellAngle").getDouble(0);
  }

  public double getPowerCellProbability(){
    return m_ntTble.getEntry("PowerCellProbability").getDouble(0);
  }

  public double getPowerCellDistance(){
    return m_ntTble.getEntry("PowerCellDistance").getDouble(0);
  }

  public double getPowerCellFrameCounter(){
    return m_ntTble.getEntry("FrameCounter").getDouble(0);
  }

  public boolean isPowerCellFound(){
      return m_ntTble.getEntry("PowerCellFound").getBoolean(false);
  }

}
