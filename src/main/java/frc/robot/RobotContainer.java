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

    // setup the shooter using the right trigger
    JoystickButton rightTrigger = new JoystickAnalogButton(m_Joystick1, 3); // the trigger associated with shooting
    rightTrigger.whenHeld(new frc.robot.commands.Shoot().withTimeout(5.0));

    // a button - capture
    JoystickButton aButton = new JoystickButton(m_Joystick1, 1); // the button associated while caputuring balls
    aButton.whenPressed(new frc.robot.commands.DeployIntake());
    aButton.whileHeld(new frc.robot.commands.IntakeCapture());
    aButton.whenReleased(new frc.robot.commands.RetractIntake());

    // left trigger - activate targeting
    JoystickButton leftTrigger = new JoystickAnalogButton(m_Joystick1, 2); // the trigger associated with targeting
    leftTrigger.whenHeld(new frc.robot.commands.Target());

    // x button - activate top indexer
    JoystickButton xButton = new JoystickButton(m_Joystick1, 3); // the buttom runs the indexer
    xButton.whileHeld(new frc.robot.commands.RunIndexer());

    // b button - clear indexer
    JoystickButton bButton = new JoystickButton(m_Joystick1, 2); // the buttom runs the indexer
    bButton.whileHeld(new frc.robot.commands.ClearIndexer());

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