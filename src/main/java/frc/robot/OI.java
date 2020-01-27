/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;

import frc.robot.subsystems.Drive;



public class OI extends SubsystemBase {
  private final Joystick m_stick = new Joystick(0);
  private final DigitalOutput m_LEDrelay = new DigitalOutput(0);
  private Drive m_drive = null;
  private int m_currentShot = 0;  // How many times have we finished shooting
  private int m_nextShot = 0;     // The count of the next shot (used to prevent multiple simultaneous shots)
  
  /**
   * Creates a new OI.
   */
  public OI() {

  }

  public void SetDrive(Drive d) {
    m_drive = d;
  }

  public Joystick getDriverStick() {
    return m_stick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Check to see if driver said shoot!
    // Right trigger
    /*  ---- Moved into Robot Container ---
    // TODO delete when done testing
    double dValueRight = m_stick.getRawAxis(3);

    if (dValueRight > 0.05) {
      if (m_nextShot == m_currentShot) {
        m_nextShot++;
        this.shoot();
      }
    } else {
      // When the trigger is release we have completed a shot
      m_currentShot++;
      this.shootStop();
    }
    */

    // Check to see if driver said capture!
    // Green A button
    boolean bValueButton;
    bValueButton  = m_stick.getRawButton(1);

    if (bValueButton){
      // System.out.println("Green A Button");
      m_LEDrelay.set(true);
      System.out.println("DigitalOutput set to true");
//      m_LEDrelay.set(Relay.Value.kForward);
//      System.out.println("Relay set to kForward");
      this.capture();
    }


    // Blue X button
    bValueButton  = m_stick.getRawButton(3);

    if (bValueButton){
      // System.out.println("Blue X Button");
      m_LEDrelay.set(false);
      // System.out.println("DigitalOutput set to false");
//      m_LEDrelay.set(Relay.Value.kOff);
//      System.out.println("Relay set to kOff");
      this.capture();
    }

    // Yellow Y button
    bValueButton  = m_stick.getRawButton(4);

    if (bValueButton){
     // System.out.println("Yellow Y Button");
  //    m_LEDrelay.set(Relay.Value.kOn);
  //    System.out.println("Relay set to kOn");
      this.capture();
    }

    // Red B button
    bValueButton  = m_stick.getRawButton(2);

    if (bValueButton){
      // System.out.println("Red B Button");
  //    m_LEDrelay.set(Relay.Value.kReverse);
  //    System.out.println("Relay set to kReverse");
      this.capture();
    }


    // check to see if driver said aim!
    double dValueLeft = m_stick.getRawAxis(2);
    if (dValueLeft > 0.05) {
    //  m_LEDrelay.set(true);
     // System.out.println("DigitalOutput set to true");
      this.aim();
    } else {
      m_drive.SetTargeting(false);
     // m_LEDrelay.set(false);
     // System.out.println("DigitalOutput set to false");
    }
  }

  // when the driver pushes the aim button
  private void aim() {
    // System.out.println("Aim!");
    m_drive.SetTargeting(true);
  }

  // when the driver pushes the shoot button
  private void shoot() {
    TheRobot.log("Driver Input says: Shooting!");
    Robot r = TheRobot.getInstance();
    r.m_shooter.shoot();
  }

  // when the driver release the shoot button
  private void shootStop() {
    TheRobot.log("Driver Inputs says: Stop Shoointg!");
    Robot r = TheRobot.getInstance();
    r.m_shooter.stop();
  }

  //when the driver pushes the A button
  private void capture() {

    // System.out.println("Capture!");
    // extends intake
    // starts capture on intake
    // start capture om indexer
  }
    //when left stick is pushed up
  private void left() {

    // System.out.println("Left foward!");
  }
  //when right stick is pushed up
  private void right() {

    // System.out.println("Right foward!");
  }


}
