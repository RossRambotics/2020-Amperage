/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.*;
import frc.robot.subsystems.Drive;
import frc.robot.*;

/**
 * Automatically find the target and end on target.
 */

public class AutoTarget extends CommandBase {
  private Drive m_drive = null;
  private ShooterLookUp m_lookUpTable = new ShooterLookUp();
  /**
   * Creates a new AutoTarget.
   */
  public AutoTarget(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.enableBrakes(false);

    Robot r = TheRobot.getInstance();
    r.m_shooter.setLEDRing(true);
    r.m_intake.extend();
    r.m_drive.SetPowerPortTargeting(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle = m_lookUpTable.getTargetAngle();
    double dFrame = m_lookUpTable.getFrameCounter();

    if (true) {
    TheRobot.log("Frame: " + TheRobot.toString(dFrame) +
                 " TargetAngle: " + TheRobot.toString(targetAngle));
    }

    m_drive.TargetDriveSpin(targetAngle, dFrame);

    return;
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.enableBrakes(true);

    Robot r = TheRobot.getInstance();
    r.m_shooter.setLEDRing(false);
    r.m_drive.SetPowerPortTargeting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
