package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElvConst;
import frc.robot.subsystems.Elevator;

/**
 * Command that allows for precise movement of the Elevator to a predetermined position.
 */
public class AutoElevator extends Command {
    private Elevator m_Elevator;
    private double angleSetPoint;
    ProfiledPIDController pid = new ProfiledPIDController(ElvConst.kP, ElvConst.kI, ElvConst.kD, new TrapezoidProfile.Constraints(ElvConst.maxVelocity, ElvConst.maxAcceleration));
    ElevatorFeedforward feedforward = new ElevatorFeedforward(ElvConst.kS, ElvConst.kG, ElvConst.kV, ElvConst.kA);

    public AutoElevator(Elevator m_Elevator, double setPoint) {
        this.m_Elevator = m_Elevator;
        angleSetPoint = setPoint;
        addRequirements(m_Elevator);
    }

    @Override
    public void initialize() {
        pid.setTolerance(ElvConst.tolerance);
        pid.reset(Units.degreesToRotations(m_Elevator.getElevatorPosition(true)));
        System.out.println("Executing [AutoElevator] command.");
    }

    @Override
    public void execute() {
        m_Elevator.setSpeed(pid.calculate(m_Elevator.getElevatorPosition(true), angleSetPoint) + feedforward.calculate(ElvConst.setpointVelocity));
    }

    @Override
    public boolean isFinished() {
        return pid.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.brake();
        if (interrupted) {
            System.out.println("!WARNING!\nCommand failure!");
        } else {
            System.out.println("Successfully completed command.");
        }
    }
}
