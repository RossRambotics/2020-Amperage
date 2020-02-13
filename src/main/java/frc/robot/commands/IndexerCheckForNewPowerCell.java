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

/*  Command monitors sensor that checks for the presence of a powercell
    to move from the intake into the indexer.  If a powercell is present
    call the command IndexNewPowerCell
*/

public class IndexerCheckForNewPowerCell extends CommandBase {
  /**
   * Creates a new IndexerCheckForNewPowerCell.
   */
  public IndexerCheckForNewPowerCell() {
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

    // TODO --- Check whether the intake is extended?

    // Check to see if we see a power cell that we need 
    // to index into therobot
    if (r.m_indexer.SenseIntakePC() == true) {
      r.m_CMDScheduler.schedule(new IndexNewPowerCell());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
