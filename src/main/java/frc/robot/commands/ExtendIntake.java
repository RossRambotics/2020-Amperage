/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.TheRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ExtendIntake extends CommandBase {
  public ExtendIntake(Subsystem indexer) {
    //super(0.25); // make this command take time

    // Use addRequirements() here to declare subsystem dependencies.
    //this.addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot r = TheRobot.getInstance();

    r.m_intake.extend(); 
    TheRobot.log("Extend Command Initialized.");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TheRobot.log("Extend Command Ending.");
    Robot r = TheRobot.getInstance();
    r.m_CMDScheduler.schedule(new RetractIntake());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
