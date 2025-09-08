package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends Command {
    private Elevator m_Elevator;
    private DoubleSupplier speedSup;

    public TeleopElevator (Elevator m_Elevator, DoubleSupplier speedSup) {
        this.m_Elevator = m_Elevator;
        this.speedSup = speedSup;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_Elevator.setSpeed(speedSup.getAsDouble());
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