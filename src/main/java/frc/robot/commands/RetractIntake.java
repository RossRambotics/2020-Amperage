package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.TheRobot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractIntake extends CommandBase {

    private boolean m_finished = false;

    public RetractIntake() { // when instance of command is created

    }

    @Override
    public void initialize() { // frist timethe command was scheduled
        TheRobot.log("RetractIntake Initializing...");
    }

    @Override
    public void execute() { // when command is running called repeatedly -- stop by changing m_finished
      Robot r = TheRobot.getInstance();
      r.m_intake.retract();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            TheRobot.log("RetractIntake interrupted...");
        } else {
            TheRobot.log("RetractIntake ended...");
        }
    }

    @Override
    public boolean isFinished() { // called often; ends commmadn when the return is true
        return m_finished;
    }
}