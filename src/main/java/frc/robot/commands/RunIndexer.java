/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.TheRobot;

public class RunIndexer extends CommandBase {
  /**
   * Creates a new RunIndexer.
   */
  public RunIndexer(Subsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(indexer);
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot r = TheRobot.getInstance();

    this.addRequirements(r.m_indexer);
    r.m_indexer.advance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot r = TheRobot.getInstance();
    r.m_indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
