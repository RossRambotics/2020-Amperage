/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.TheRobot;
import frc.robot.eRobotSide;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RetractClimbLift extends CommandBase {

    private eRobotSide m_winch;
    public RetractClimbLift(eRobotSide w) {
      m_winch = w;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      TheRobot.log("Climb Lift Retract Initializing...");
      Robot r = TheRobot.getInstance(); // calls the rectract function on the climber
      r.m_climber.releaseLift(m_winch);
    }
  
    @Override
    public void end(boolean interrupted)
    {
        Robot r = TheRobot.getInstance();
        r.m_climber.stopLift(m_winch);
    }
  
    @Override
    public boolean isFinished() { // called often; ends commmadn when the return is true
        return false;
    }
}
