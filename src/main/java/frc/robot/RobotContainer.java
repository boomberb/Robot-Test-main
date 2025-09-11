// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OprConst;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Mailbox;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopMailbox;

public class RobotContainer {
  private final Joystick controller = new Joystick(OprConst.kDriverControllerPort);
  private final Elevator m_Elevator = new Elevator();
  private final Mailbox m_Mailbox = new Mailbox();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OprConst.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    m_Mailbox.setDefaultCommand(new TeleopMailbox(m_Mailbox, () -> weapons.getRawAxis(mailbox))){

    }
    })
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
