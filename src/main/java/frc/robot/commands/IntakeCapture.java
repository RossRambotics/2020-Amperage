package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.TheRobot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCapture extends CommandBase {

    private boolean m_finished = false;

    public IntakeCapture() { // when instance of command is created

    }

    @Override
    public void initialize() { // frist timethe command was scheduled
        TheRobot.log("IntakeCapture Initializing...");
    }

    @Override
    public void execute() { // when command is running called repeatedly -- stop by changing m_finished
      Robot r = TheRobot.getInstance();
      r.m_intake.capture();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            TheRobot.log("IntakeCapture interrupted...");
        } else {
            TheRobot.log("IntakeCapture ended...");
        }

        Robot r = TheRobot.getInstance();
        r.m_intake.stopCapture();
    }

    @Override
    public boolean isFinished() { // called often; ends commmadn when the return is true
        return m_finished;
    }
}