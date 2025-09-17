package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Payload;

public class SelfDestruct extends Command {
    private Payload m_Payload;

    public SelfDestruct (Payload m_Payload) {
        this.m_Payload = m_Payload;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_Payload.detonate();
    }
}
