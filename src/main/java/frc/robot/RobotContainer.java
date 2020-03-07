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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private Joystick m_DriverStick = new Joystick(0);
  private Joystick m_OperatorStick = new Joystick(1); // operators joystick

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
    CommandBase c;

    // setup the shooter using the right trigger
    JoystickButton rightTrigger = new JoystickAnalogButton(m_DriverStick, 3); // the trigger associated with shooting
    c = new SequentialCommandGroup(
        new ClearIndexer(r.m_indexer).withTimeout(0.05),
        new ReadyShooter(r.m_indexer).withTimeout(0.2),
        new frc.robot.commands.Shoot(r.m_indexer, r.m_shooter).withTimeout(4.0));
    //c = new frc.robot.commands.Shoot(r.m_indexer).withTimeout(10.0);

    rightTrigger.whenPressed(c);

    // a button - capture
    JoystickButton aButton = new JoystickButton(m_DriverStick, 1); // the button associated while caputuring balls
    aButton.toggleWhenPressed(new frc.robot.commands.IntakeCapture(r.m_intake), true);

    // b  button - capture
    JoystickButton bButton = new JoystickButton(m_DriverStick, 2); // the button associated while cleraing the balls out of the intake
    bButton.whenHeld(new frc.robot.commands.ClearIntake(r.m_intake), true);

    // x button - capture
    JoystickButton xButton = new JoystickButton(m_DriverStick, 3); // the button associated while caputuring balls
    xButton.toggleWhenPressed(new frc.robot.commands.ExtendIntake(r.m_indexer), true);

    // left trigger - activate targeting
    JoystickButton leftTrigger = new JoystickAnalogButton(m_DriverStick, 2, 0.05); // the trigger associated with targeting
    leftTrigger.whenHeld(new frc.robot.commands.Target());

    // start button - activate top indexer
    JoystickButton startButton = new JoystickButton(m_DriverStick, 8); // the button runs the indexer
    startButton.whileHeld(new frc.robot.commands.RunIndexer(r.m_indexer));

    // back button - clear indexer
    JoystickButton backButton = new JoystickButton(m_DriverStick, 7); // the buttom runs the indexer
    backButton.whileHeld(new frc.robot.commands.ClearIndexer(r.m_indexer));

    // right shoulder button - clear shooter
    JoystickButton rightShoulderButton = new JoystickButton(m_DriverStick, 6);
    //rightShoulderButton.whileHeld(new frc.robot.commands.CompactShooter());
    c = new SequentialCommandGroup(
        new ClearIndexer(r.m_indexer).withTimeout(0.05),
        new ReadyShooter(r.m_indexer).withTimeout(0.1)
    );
    rightShoulderButton.whenPressed(c);

    // Click left stick to toggle gears
    JoystickButton leftStickClick = new JoystickButton(m_DriverStick, 9);
    leftStickClick.whenPressed(new ToggleSlowDrive());
    
    //click Y to target low Power Port
    JoystickButton yButton = new JoystickButton(m_DriverStick, 4);
    yButton.whenPressed(new ToggleLowPowerPort());

    /** 
     * configure operator joystick buttons
     */
     
    // start button - activate top indexer
    JoystickButton startButtonOperator = new JoystickButton(m_OperatorStick, 8); // the button runs the indexer
    startButtonOperator.whileHeld(new frc.robot.commands.RunIndexer(r.m_indexer));
    
    // back button - clear indexer
    JoystickButton backButtonOperator = new JoystickButton(m_OperatorStick, 7); // the buttom runs the indexer
    backButtonOperator.whileHeld(new frc.robot.commands.ClearIndexer(r.m_indexer));

    // Winch up / Retract right side
    JoystickButton operatorRightTrigger = new JoystickAnalogButton(m_OperatorStick, 3);
    operatorRightTrigger.whileHeld(new frc.robot.commands.RetractClimbWinch(eRobotSide.RIGHT));
    
    // Winch up / Retract left side
    JoystickButton operatorLeftTrigger = new JoystickAnalogButton(m_OperatorStick, 2);
    operatorLeftTrigger.whileHeld(new frc.robot.commands.RetractClimbWinch(eRobotSide.LEFT));
    
    // Operator can cancel the shoot / clear the shooter
    JoystickButton opRightShoulderButton = new JoystickButton(m_OperatorStick, 6);
    c = new SequentialCommandGroup(
        new ClearIndexer(r.m_indexer).withTimeout(0.05),
        new ReadyShooter(r.m_indexer).withTimeout(0.1)
    );
    opRightShoulderButton.whenPressed(c);

    /** 
     * Configure operator nudge commands to help with hanging
     */
    double dNudgeTime = 0.2;  // Nudge time
    double dNudgePower = 0.2; // Nudge motor power

    // Nudge forward
    POVButton operatorNudgeForward = new POVButton(m_OperatorStick, 0);
    operatorNudgeForward.whenPressed(new DriveNudge(r.m_drive, dNudgePower, dNudgePower).withTimeout(dNudgeTime));

    // Nudge backward
    POVButton operatorNudgeBackward = new POVButton(m_OperatorStick, 180);
    operatorNudgeBackward.whenPressed(new DriveNudge(r.m_drive, -dNudgePower, -dNudgePower).withTimeout(dNudgeTime));

    // Nudge right back
    POVButton operatorNudgeRightBack = new POVButton(m_OperatorStick, 90);
    operatorNudgeRightBack.whenPressed(new DriveNudge(r.m_drive, 0, -dNudgePower).withTimeout(dNudgeTime));

    // Nudge left back
    POVButton operatorNudgeLeftBack = new POVButton(m_OperatorStick, 270);
    operatorNudgeLeftBack.whenPressed(new DriveNudge(r.m_drive, -dNudgePower, 0).withTimeout(dNudgeTime));
  }

  public Joystick getDriverStick() {
    return m_DriverStick;
  }
}