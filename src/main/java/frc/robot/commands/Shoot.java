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
    Robot r = TheRobot.getInstance();
    addRequirements(r.m_indexer);
    addRequirements(r.m_shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();
    r.m_shooter.shoot();
    r.m_hood.extend();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      TheRobot.log("Shoot interrupted...");
    } else {
      TheRobot.log("Shoot ended...");
    }
    Robot r = TheRobot.getInstance();
    r.m_shooter.stop();
    r.m_hood.retract();
    r.m_indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
