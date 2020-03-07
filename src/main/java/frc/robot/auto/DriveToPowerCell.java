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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PowerCellTargeter;

public class DriveToPowerCell extends CommandBase {
  /**
   * Creates a new DriveStraight.
   */

  private int m_orignalDriveStyle = 0;
  private PowerCellTargeter m_Targeter;
  private double m_lastDistance = 2;
  private double m_velocity = 0;

  public DriveToPowerCell(Drive drive, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Targeter = new PowerCellTargeter();
    TheRobot.log("Initailizing Drive To PowerCell");
    Robot r = TheRobot.getInstance();
    this.addRequirements(r.m_drive);

    m_velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot r = TheRobot.getInstance();
    r.m_drive.SetPowerCellTargeting(true);
    
    m_orignalDriveStyle = r.m_drive.m_DriveStyle;
    r.m_drive.m_DriveStyle = 0;
    m_lastDistance = 2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();
    r.m_drive.PowerCellTargetDrive(m_velocity, m_velocity, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot r = TheRobot.getInstance();
    r.m_drive.SetPowerCellTargeting(false);
    r.m_drive.m_DriveStyle = m_orignalDriveStyle;
    TheRobot.log("DriveToPowerCell: Ending.");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Robot r = TheRobot.getInstance();

    if((m_Targeter.getPowerCellDistance() - 0.05) < m_lastDistance && m_Targeter.isPowerCellFound()){
      m_lastDistance = m_Targeter.getPowerCellDistance();
      return false;
    }
    TheRobot.log("DriveToPowerCell: Ready to capture!");
    return true;
  }
}
