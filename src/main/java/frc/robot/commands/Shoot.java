/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.TheRobot;

public class Shoot extends CommandBase {

  private boolean m_finished = false;
  /**
   * Creates a new Shoot.
   */
  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TheRobot.log("Shoot Initializing...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();
    
    // get distance to target
    double d = r.m_powerPowerTargeter.getDistance();

    // tell shooter to come up to target speed based on distance
  
    if (r.m_shooter.ready(d)) {
      // start the indexer
      r.m_indexer.shoot();
    } else {
      // stop the indexer
      r.m_indexer.stop();
    }

    // End the shoot command after 2 seconds
    if (r.m_CMDScheduler.timeSinceScheduled(this) > 2.0) {
      TheRobot.log("Shoot timeout hit... ending.");
      m_finished = true;
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot r = TheRobot.getInstance();
    r.m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
