/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Stick;

public class Rumble extends CommandBase {
  RumbleType m_rumbleType;
  Stick m_stick;

  /**
   * Creates a new Rumble.
   */
  public Rumble(Stick stick, RumbleType type) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_stick = stick;
    this.addRequirements(m_stick);
    m_rumbleType = type;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stick.setRumble(m_rumbleType);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_stick.clearRumble(m_rumbleType);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
