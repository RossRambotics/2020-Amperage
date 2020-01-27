/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Color Sensor classes
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;


public class ControlPanel extends SubsystemBase {

  // set up color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.188, 0.444, 0.367);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final Color kUnknownTarget = ColorMatch.makeColor(0.438,0.434,0.138);


  /**
   * Creates a new ControlPanel.
   */
  public ControlPanel() {
    // add colors to color matcher
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    
  }
  // extends arm that turns color wheel
  // Returns false if not fully extended
  // Returns true if fully extended
  public boolean extened() {
    return false;
  }

  // retracts arm that turns color wheel
  // Returns false if not fully retracts
  // Returns true if fully retracts
  public boolean retract() {
    return false;
  }

  // spins the color wheel "turner"
  // Returns false if not curently spinning
  // Returns true if curently spinning
  public boolean spin() {
    return false;
  }

  // turns wheel to a specific color 
  // Returns false if not curently on selected color
  // Returns true if curently on selected color
  public boolean color() {
    return false;
  }

  private String getColor() {
    // doing color stuff
    final Color detectedColor = m_colorSensor.getColor();
    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

  if (match.color == kBlueTarget)  {
    colorString = "Blue";
  } else if (match.color == kRedTarget) {
    colorString = "Red";
  } else if (match.color == kGreenTarget) {
    colorString = "Green";
  } else if (match.color == kYellowTarget) {
    colorString = "Yellow";
  } else if (match.color == kUnknownTarget) {
    colorString = "UNKNOWN";
  } else {
    colorString = "Mr Jones";
  }
  return colorString;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
