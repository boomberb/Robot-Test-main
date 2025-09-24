// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoMailbox;
import frc.robot.commands.SelfDestruct;
import frc.robot.commands.TeleopMailbox;
import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.OprConst;
//import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mailbox;
import frc.robot.subsystems.Payload;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  
  //Subsystems
  //private final Elevator m_Elevator = new Elevator();
  private final Mailbox m_Mailbox = new Mailbox();
  private final Payload m_Payload = new Payload();
  private final Swerve s_Swerve = new Swerve();

  //Controllers
  private final Joystick driver = new Joystick(OprConst.kDriverControllerPort);
  private final Joystick weapons = new Joystick(OprConst.kDriverControllerPort);

  //Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value; // Forward and backward controls
  private final int strafeAxis = XboxController.Axis.kLeftX.value; // Right and left controls
  private final int rotationAxis = XboxController.Axis.kRightX.value; // Rotation controls

  //Drive Buttons
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  //Weapon Controls
  //private final int elevatorManual = XboxController.Axis.kLeftY.value;
  private final int mailboxManual = XboxController.Axis.kRightY.value;

  //Weapon Buttons
  private final JoystickButton intake = new JoystickButton(weapons, XboxController.Button.kLeftBumper.value);
  private final JoystickButton score = new JoystickButton(weapons, XboxController.Button.kRightBumper.value);
  private final JoystickButton detonate = new JoystickButton(weapons, XboxController.Button.kB.value);

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> -driver.getRawAxis(rotationAxis), 
                    () -> robotCentric.getAsBoolean()
                )
            );
    
    // m_Elevator.setDefaultCommand(
    //   new TeleopElevator(
    //     m_Elevator,
    //     () -> -weapons.getRawAxis(elevatorManual)
    //   )
    // );

    m_Mailbox.setDefaultCommand(
      new TeleopMailbox(
        m_Mailbox,
        () -> -weapons.getRawAxis(mailboxManual)
      )
    );

    // Configure the button bindings
    configureBindings();
  }

  private void configureBindings() {
    // Driver Buttons
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    // Weapons Buttons
    score.whileTrue(new AutoMailbox(m_Mailbox, true));
    intake.whileTrue(new AutoMailbox(m_Mailbox, false));
    detonate.onTrue(new SelfDestruct(m_Payload));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}