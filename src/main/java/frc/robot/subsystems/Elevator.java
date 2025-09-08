package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConst;

public class Elevator extends SubsystemBase {
    public TalonFX elevatorMotor;

    public Elevator () {
        elevatorMotor = new TalonFX(ElevatorConst.elevatorMotorID);
    }

    public void setSpeed (double output) {
        elevatorMotor.setVoltage(output);
    }

    public void brake () {
        elevatorMotor.setVoltage(0);
    }
}
