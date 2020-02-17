/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.helper.JoystickAnalogButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Command m_autoCommand = null;
  private Joystick m_Joystick1 = new Joystick(0);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    TheRobot.log("RobotContainer Created.");
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Robot r = TheRobot.getInstance();

    // setup the shooter using the right trigger
    JoystickButton rightTrigger = new JoystickAnalogButton(m_Joystick1, 3); // the trigger associated with shooting
    rightTrigger.whenHeld(new frc.robot.commands.Shoot(r.m_indexer).withTimeout(10.0));

    // a button - capture
    JoystickButton aButton = new JoystickButton(m_Joystick1, 1); // the button associated while caputuring balls
    aButton.toggleWhenPressed(new frc.robot.commands.IntakeCapture(r.m_intake), true);

    // b  button - capture
    JoystickButton bButton = new JoystickButton(m_Joystick1, 2); // the button associated while cleraing the balls out of the intake
    bButton.whenHeld(new frc.robot.commands.ClearIntake(r.m_intake), true);

    // x button - capture
    JoystickButton xButton = new JoystickButton(m_Joystick1, 3); // the button associated while caputuring balls
    xButton.toggleWhenPressed(new frc.robot.commands.ExtendIntake(r.m_indexer), true);

    // left trigger - activate targeting
    JoystickButton leftTrigger = new JoystickAnalogButton(m_Joystick1, 2); // the trigger associated with targeting
    leftTrigger.whenHeld(new frc.robot.commands.Target());

    // start button - activate top indexer
    JoystickButton startButton = new JoystickButton(m_Joystick1, 8); // the button runs the indexer
    startButton.whileHeld(new frc.robot.commands.RunIndexer(r.m_indexer));

    // back button - clear indexer
    JoystickButton backButton = new JoystickButton(m_Joystick1, 7); // the buttom runs the indexer
    backButton.whileHeld(new frc.robot.commands.ClearIndexer(r.m_indexer));

    // right shoulder button - clear shooter
    JoystickButton rightShoulderButton = new JoystickButton(m_Joystick1, 6);
    rightShoulderButton.whileHeld(new frc.robot.commands.CompactShooter());

    /*
    JoystickButton leftShoulder = new JoystickButton(m_Joystick1, 5); // runs the lift down
    leftShoulder.whenPressed(new frc.robot.commands.RetractClimbLift());

    JoystickButton rightShoulder = new JoystickButton(m_Joystick1, 6); // runs the lift up
    rightShoulder.whenPressed(new frc.robot.commands.ExtendClimbLift());

    JoystickButton backButton = new JoystickButton(m_Joystick1, 7); // releases the winch down
    backButton.whileHeld(new frc.robot.commands.ReleaseClimbWinch());
    
    JoystickButton startButton = new JoystickButton(m_Joystick1, 8); // releases the winch down
    startButton.whileHeld(new frc.robot.commands.RetractClimbWinch());
    */

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}