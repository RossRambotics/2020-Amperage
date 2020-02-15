/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*  runs the indexer until the Power Cell Intake sensor
    no longer sees a power cell.  Hopefully this indexs
    the next power cell!
*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.TheRobot;

public class IndexNewPowerCell extends CommandBase {

  /**
   * Creates a new IndexNewPowerCell.
   */
  public IndexNewPowerCell() {
    // Use addRequirements() here to declare subsystem dependencies.
    Robot r = TheRobot.getInstance();
    this.addRequirements(r.m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();

    // start trying to index a power cell
    r.m_indexer.advance();
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

    // Check to see if we see a power cell that we need 
    // to index into the robot
    if (r.m_indexer.SenseIntakePC() == false) {
      // the ball either was indexed or not...
      r.m_indexer.stop();
      return true;
    }

    return false;
  }
}
