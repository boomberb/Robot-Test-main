package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * Command that allows for precise movement of the Elevator to a predetermined position.
 */
public class AutoElevator extends Command {
    private Elevator m_Elevator;

    public AutoElevator(Elevator m_Elevator) {
        this.m_Elevator = m_Elevator;

        addRequirements(m_Elevator);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
