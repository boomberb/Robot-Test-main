package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mailbox;

public class AutoMailbox extends Command {
    private Mailbox m_Mailbox;

    public AutoMailbox (Mailbox m_Mailbox) {
        this.m_Mailbox = m_Mailbox;
        addRequirements(m_Mailbox);
    }
}
