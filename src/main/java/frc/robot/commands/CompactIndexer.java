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

// moves the power cells in the indexer past the 1st sensor
// allowing room for next power cell to be inserted
public class CompactIndexer extends CommandBase {

  /**
   * Creates a new CompactIndexer.
   */
  public CompactIndexer(Subsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();
    boolean b = r.m_indexer.SenseIndex0();

    if (b) {
      r.m_indexer.compact();
    } else {
      r.m_indexer.stop();
    }
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
    Robot r = TheRobot.getInstance();
    boolean b = r.m_indexer.SenseIndex0();

    return !b;
  }
}
