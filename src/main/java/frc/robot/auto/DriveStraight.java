/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.TheRobot;

public class DriveStraight extends CommandBase {
  double m_dist = 0;
  double dStart = 0;
  private double m_dVelocity;
  /**
   * Creates a new DriveStraight.
   */
  public DriveStraight(double dist, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dist = dist * 45000.0;
    m_dVelocity = velocity;
    TheRobot.log("Initailizing Drive Straight");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot r = TheRobot.getInstance();
    dStart = r.m_drive.getLeftEncoderPosition();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();
    r.m_drive.moveAtVelocity(m_dVelocity, m_dVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Robot r = TheRobot.getInstance();
    if (Math.abs(r.m_drive.getLeftEncoderPosition() - dStart) < m_dist){
    TheRobot.log(TheRobot.toString(Math.abs(r.m_drive.getLeftEncoderPosition() - dStart) - m_dist));

      return false;
    }
    else 
      return true;
  }
}
