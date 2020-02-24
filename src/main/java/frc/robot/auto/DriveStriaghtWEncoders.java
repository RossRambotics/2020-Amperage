package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.*;
import frc.robot.subsystems.Drive;
import frc.robot.*;

/**
 * Drive Straight for a distance
 */

public class DriveStriaghtWEncoders extends CommandBase {
                                                /* 21000 = 1 wheel rotation
                                                 * wheel is 4 in diameter
                                                 * 39.37 in per meter
                                                 */
    private double m_clickPerMeter = 21000.0 * 4.0 * 3.1415; 
    private double m_chokeClicks = 4000; // when to begin to halt the robot
    private double m_correctionCoefficent = .02;
    private double m_exponentialReduction = 1.2;

    private double m_targetDistance = 0; // the target distance to be driven in meters
    private double m_targetClicks = 0; // the target distance in clicks
    private double m_maxVelocity = 0;
    
    private double m_intialRightEncoderPosition = 0;
    private double m_intialLeftEncoderPosition = 0;

    private boolean m_isFinished = false; // when the command is finshed

    private Drive m_drive = null;
    private ShooterLookUp m_lookUpTable = new ShooterLookUp();
    
    /**
     * Creates a DriveStraightWEncoder Commands
     * drive -> drive subsystem
     * distance -> distance to travel in meters
     * velocity -> the max velocity to travel at
     */
    public DriveStriaghtWEncoders(Drive drive, double distance, double velocity) {
      // Use addRequirements() here to declare subsystem dependencies.
      m_drive = drive;
      addRequirements(m_drive);

      m_targetDistance = distance;
      m_targetClicks = m_targetDistance * m_clickPerMeter;

      m_maxVelocity = velocity;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_drive.SetUseJoystick(false);
      m_drive.enableBrakes(false);

      m_intialLeftEncoderPosition = m_drive.getLeftEncoderPosition();
      m_intialRightEncoderPosition = m_drive.getRightEncoderPosition();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(absoluteClicksRemaining() < 10)
        {
            m_isFinished = true;
            m_drive.moveAtVelocity(0, 0);
            return;

        }
        
        double[] velocities = polarizeVelociies(sraightenVelocities(getStraightVelocity()));
        //left = [0]
        //right = [1]
        
        m_drive.moveAtVelocity(velocities[0], velocities[1]);
        
      return;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_drive.enableBrakes(true);
      m_drive.SetUseJoystick(true);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return m_isFinished;
    }

    private double getStraightVelocity() // gets velocity base on distance remianing
    {
        double velocityTarget = m_maxVelocity;

        double distanceRemaining = absoluteClicksRemaining(); 
        if(distanceRemaining < m_chokeClicks)
        {
            velocityTarget = velocityTarget * Math.pow((distanceRemaining / m_chokeClicks), m_exponentialReduction); // slows the bot as it approachs the target
        }

        return velocityTarget;
    }

    private double[] sraightenVelocities(double straightVelocity) // straightens the robot by modifing the velocities
    {
        double leftVelocity = straightVelocity;
        double rightVelocity = straightVelocity;

        double encoderOffset = Math.abs(m_drive.getLeftEncoderPosition() - m_intialLeftEncoderPosition) - Math.abs(m_drive.getRightEncoderPosition() - m_intialRightEncoderPosition);
        // is positive = left is ahead
        // is negative = right is ahead
        if(encoderOffset > 0)
        {
            double velocityDecrease = straightVelocity * (Math.pow(encoderOffset, 0.5) * m_correctionCoefficent);    
            if(velocityDecrease > 0.9 * straightVelocity)
            {
                velocityDecrease = 0.9 * straightVelocity;
            }

            leftVelocity = leftVelocity - velocityDecrease;
        }else{
            double velocityDecrease = straightVelocity * (Math.pow(-encoderOffset, 0.5) * m_correctionCoefficent);    
            if(velocityDecrease > 0.9 * straightVelocity)
            {
                velocityDecrease = 0.9 * straightVelocity;
            }

            rightVelocity = rightVelocity - velocityDecrease;
        }


        double[] velocityArray = {leftVelocity, rightVelocity};
        return velocityArray;
    }

    private double[] polarizeVelociies(double[] velocities) // takes into account the direction of travel
    {
        double[] velocitiesPolarized = {0, 0};

        if(clicksRemainging() < 0)
        {
            velocitiesPolarized[0] = velocities[0] * -1;
            velocitiesPolarized[1] = velocities[1] * -1;
        }else{
            velocitiesPolarized[0] = velocities[0];
            velocitiesPolarized[1] = velocities[1];
        }

        return velocitiesPolarized;
    }

    private double clicksRemainging() // returns the remaining clicks the robot has to travel
    {
        double leftRemaining = m_targetClicks - (m_drive.getLeftEncoderPosition() - m_intialLeftEncoderPosition);
        double rightRemaining = m_targetClicks - (m_drive.getRightEncoderPosition() - m_intialRightEncoderPosition);

        System.out.println((leftRemaining + rightRemaining) / 2);

        return (leftRemaining + rightRemaining) / 2;
    }

    private double absoluteClicksRemaining()
    {
        return Math.abs(clicksRemainging());
    }
}