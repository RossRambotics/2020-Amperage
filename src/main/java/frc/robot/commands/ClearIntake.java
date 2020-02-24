/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.TheRobot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClearIntake extends CommandBase {

  private boolean m_finished = false;
  /**
   * Creates a new ClearIntake.
   */
  public ClearIntake(Subsystem intake) {
    this.addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TheRobot.log("ClearIntake Initializing...");
        Robot r = TheRobot.getInstance();
        if (r.m_intake.isExtended()) {
            m_finished = false;
            r.m_intake.clear();

        } else
            m_finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();
    if (!r.m_intake.isExtended()) {
        m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot r = TheRobot.getInstance();
    r.m_intake.stopCapture();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
