package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.TheRobot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeCapture extends CommandBase {

    private boolean m_finished = false;

    public IntakeCapture(Subsystem intake) { // when instance of command is created
        this.addRequirements(intake);
    }

    @Override
    public void initialize() { // frist time the command was scheduled
        TheRobot.log("IntakeCapture Initializing...");
        Robot r = TheRobot.getInstance();
        if (r.m_intake.isExtended()) {
            m_finished = false;
            r.m_intake.capture();

        } else
            m_finished = true;
    }

    @Override
    public void execute() { // when command is running called repeatedly -- stop by changing m_finished
        Robot r = TheRobot.getInstance();
        if (!r.m_intake.isExtended()) {
            m_finished = true;
        }
       
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