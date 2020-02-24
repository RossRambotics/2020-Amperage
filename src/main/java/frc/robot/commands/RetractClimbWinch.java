package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.TheRobot;
import frc.robot.eRobotSide;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractClimbWinch extends CommandBase {

    private boolean m_finished = false;
    private eRobotSide m_winch;

    public RetractClimbWinch(eRobotSide w) { // when instance of command is created
        Robot r = TheRobot.getInstance(); 
        this.addRequirements(r.m_climber);

        m_winch = w;
    }

    @Override
    public void initialize() { // frist time the command was scheduled
        TheRobot.log("Climb Winch Retract Initializing...");
        Robot r = TheRobot.getInstance(); 
        r.m_climber.retractWinch(m_winch);
    }

    @Override
    public void execute() { // when command is running called repeatedly -- stop by changing m_finished

    }

    @Override
    public void end(boolean interrupted)
    {
        Robot r = TheRobot.getInstance();
        r.m_climber.stopWinch(m_winch);
    }

    @Override
    public boolean isFinished() { // called often; ends commmadn when the return is true
        return m_finished;
    }
}