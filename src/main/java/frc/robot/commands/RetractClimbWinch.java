package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.TheRobot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractClimbWinch extends CommandBase {

    private boolean m_finished = false;

    public RetractClimbWinch() { // when instance of command is created

    }

    @Override
    public void initialize() { // frist time the command was scheduled
        TheRobot.log("Climb Winch Retract Initializing...");
    }

    @Override
    public void execute() { // when command is running called repeatedly -- stop by changing m_finished
        Robot r = TheRobot.getInstance(); // calls the rectract function on the climber
        r.m_climber.retractWinch();
    }

    @Override
    public void end(boolean interrupted)
    {
        Robot r = TheRobot.getInstance();
        r.m_climber.stopRetractWinch();
    }

    @Override
    public boolean isFinished() { // called often; ends commmadn when the return is true
        return m_finished;
    }
}