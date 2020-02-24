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
import frc.robot.subsystems.Indexer;

public class ReadyShooter extends CommandBase {
  /**
   * Creates a new ReadyShooter.
   */
  public ReadyShooter(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot r = TheRobot.getInstance();
    if (!r.m_shooter.getReadyToShoot()) {
      r.m_shooter.compact();
      //r.m_CMDScheduler.schedule(new ClearIndexer(r.m_indexer).withTimeout(0.05));
    }
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot r = TheRobot.getInstance();
    r.m_shooter.stop();
    r.m_shooter.setReadyToShoot(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
