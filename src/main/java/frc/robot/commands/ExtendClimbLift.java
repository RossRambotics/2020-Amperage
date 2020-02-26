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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

//TODO dead code

public class ExtendClimbLift extends CommandBase {

  private Joystick m_OperatorStick = null;; // Operator joystick
  private double m_deadzone = 0.05;

  public ExtendClimbLift(Subsystem climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(climber);

    m_OperatorStick =  new Joystick(1); // Operator joystick
  }  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TheRobot.log("Climb Lift Extend Initializing...");
  }

  @Override
  public void execute()
  {
    Robot r = TheRobot.getInstance();
    double dvalueLYAxis = -m_OperatorStick.getRawAxis(1); // joystick Y axis is inverted
    double dvalueRYAxis = -m_OperatorStick.getRawAxis(5); // joystick Y axis is inverted
    //TheRobot.log("LiftPower " + TheRobot.toString(dvalueLYAxis));

    // enforce deadzone on joysticks
    if (Math.abs(dvalueLYAxis) < m_deadzone) dvalueLYAxis = 0;
    if (Math.abs(dvalueRYAxis) < m_deadzone) dvalueRYAxis = 0;

    if(dvalueLYAxis == 0.0){
      r.m_climber.stopLift(eRobotSide.LEFT);
      //r.m_climber.syncExtend(eRobotSide.LEFT, dvalueLYAxis);
    }else{
      r.m_climber.extendLift(eRobotSide.LEFT, dvalueLYAxis);
    }

    if(dvalueRYAxis == 0.0){
      r.m_climber.stopLift(eRobotSide.RIGHT);

      //r.m_climber.syncExtend(eRobotSide.RIGHT, dvalueRYAxis);
    }else{
      //TheRobot.log("LiftPower " + TheRobot.toString(dvalueRYAxis));
      r.m_climber.extendLift(eRobotSide.RIGHT, dvalueRYAxis);
    }
  }


  @Override
  public void end(boolean interrupted)
  {

  }

  @Override
  public boolean isFinished() { // called often; ends commmadn when the return is true
      return false;
  }
}
