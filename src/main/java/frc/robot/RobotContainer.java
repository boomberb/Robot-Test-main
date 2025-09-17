// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.OprConst;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  
  //Subsystems
  private final Elevator m_Elevator = new Elevator();
  private final Mailbox m_Mailbox = new Mailbox();
  private final Payload m_Payload = new Payload();

  //Controllers
  private final Joystick weapons = new Joystick(OprConst.kDriverControllerPort);

  //Controls
  private final int elevatorManual = XboxController.Axis.kLeftY.value;
  private final int mailboxManual = XboxController.Axis.kRightY.value;

  //Buttons
  private final JoystickButton intake = new JoystickButton(weapons, XboxController.Button.kLeftBumper.value);
  private final JoystickButton score = new JoystickButton(weapons, XboxController.Button.kRightBumper.value);

  public RobotContainer() {
    m_Elevator.setDefaultCommand(
      new TeleopElevator(
        m_Elevator,
        () -> -weapons.getRawAxis(elevatorManual)
      )
    );

    m_Mailbox.setDefaultCommand(
      new TeleopMailbox(
        m_Mailbox,
        () -> -weapons.getRawAxis(mailboxManual)
      )
    );
    configureBindings();
  }

  private void configureBindings() {
    score.whileTrue(new AutoMailbox(m_Mailbox, true));
    intake.whileTrue(new AutoMailbox(m_Mailbox, false));
  }

  public Command getAutonomousCommand() {
    return new SelfDestruct(m_Payload);
  }
}
