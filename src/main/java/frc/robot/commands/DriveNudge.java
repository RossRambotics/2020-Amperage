/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveNudge extends CommandBase {
  private Drive m_drive = null;
  double m_left, m_right;
  /**
   * Creates a new DriveNudge.
   */
  public DriveNudge(Drive drive, double left, double right) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_left = left;
    m_right = right;
    this.addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.SetPowerPortTargeting(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.NudgeDrive(m_left, m_right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.SetPowerPortTargeting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
