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

public class ConsumePowerCell extends CommandBase {
  /**
   * Creates a new DriveStraight.
   */

  private double m_velocity = 0;

  public ConsumePowerCell(Drive drive, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_velocity = velocity;
    TheRobot.log("Initailizing Drive To PowerCell");
    Robot r = TheRobot.getInstance();
    this.addRequirements(r.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot r = TheRobot.getInstance();
    r.m_drive.SetUseJoystick(false);
    r.m_intake.extend();
    r.m_intake.capture();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();
    r.m_drive.NudgeDrive(m_velocity, m_velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot r = TheRobot.getInstance();
    r.m_drive.SetUseJoystick(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
