package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.MbxConst;
import frc.robot.subsystems.Mailbox;

/**
 * Command that allows for automatic intake and deposit of the gamepiece.
 */
public class AutoMailbox extends Command {
    private Mailbox m_Mailbox;
    private boolean isScoring;

    public AutoMailbox(Mailbox m_Mailbox, boolean isScoring) {
        this.m_Mailbox = m_Mailbox;
        this.isScoring = isScoring;
        addRequirements(m_Mailbox);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        //sets the speed depending on if the mailbox is intaking or scoring
        if (isScoring) {
            m_Mailbox.setSpeed(MbxConst.scoringVoltage); //mailbox is scoring
        } else {
            m_Mailbox.setSpeed(MbxConst.intakingVoltage); //mailbox is intaking
        }
    }

    @Override
    public boolean isFinished() {
        //attempts to score if this value is true
        if (isScoring) {
            //continues operation until calculated that the mailbox cannot score
            if (m_Mailbox.canScore()) {
                return false;
            } else {
                return true;
            }
        //attempts to intake otherwise
        } else {
            //continues operation until the gamepiece is detected in the mailbox
            if (m_Mailbox.isCoralDetected()) {
                return true;
            } else {
                return false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_Mailbox.brake();
    }
}
