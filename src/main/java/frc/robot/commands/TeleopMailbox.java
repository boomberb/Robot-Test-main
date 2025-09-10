package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mailbox;

/**
 * Manual control of the mailbox [COMPLETE]
 */
public class TeleopMailbox extends Command {
    private Mailbox m_Mailbox;
    private DoubleSupplier speedSup;

    public TeleopMailbox (Mailbox m_Mailbox, DoubleSupplier speedSup) {
        this.m_Mailbox = m_Mailbox;
        this.speedSup = speedSup;
        addRequirements(m_Mailbox);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_Mailbox.setSpeed(speedSup.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Mailbox.brake();
    }
}
