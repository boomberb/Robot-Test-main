package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/**
 * Command that counteracts gravity to anchor the Elevator to a given position.
 * Include a timeout when utilizing this command or elevator will no longer move until the code is restarted.
 */
public class FixElevator extends Command {
    private Elevator m_Elevator;

    public FixElevator(Elevator m_Elevator) {
        this.m_Elevator = m_Elevator;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_Elevator.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.brake();
    }
}
