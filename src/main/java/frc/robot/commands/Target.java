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
import frc.robot.helper.ShooterValueSet;

public class Target extends CommandBase {
  private SpinUpShooter m_SpinUpShooterCMD = null;

  /**
   * Creates a new Target.
   */
  public Target() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot r = TheRobot.getInstance();
    r.m_shooter.setLEDRing(true);
    r.m_drive.SetPowerPortTargeting(true);
    TheRobot.log("Starting Targeting.");

    // TODO test this!
    m_SpinUpShooterCMD = new SpinUpShooter(r.m_shooter);
    m_SpinUpShooterCMD.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot r = TheRobot.getInstance();
    r.m_shooter.setLEDRing(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot r = TheRobot.getInstance();
    r.m_drive.SetPowerPortTargeting(false);
    r.m_shooter.setLEDRing(false);
    m_SpinUpShooterCMD.cancel();
    TheRobot.log("Ending Targeting.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
