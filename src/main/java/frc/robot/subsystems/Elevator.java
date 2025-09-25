package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElvConst;
import frc.robot.constants.SwvConst;

/**
 * Represents the Elevator subsystem and its possible functions.
 */
public class Elevator extends SubsystemBase {
    public TalonFX elevatorMotorOne;
    public TalonFX elevatorMotorTwo;

    public Elevator() {
        elevatorMotorOne = new TalonFX(ElvConst.motorID, SwvConst.swerveCANivoreName);
        elevatorMotorTwo = new TalonFX(ElvConst.motorID, SwvConst.swerveCANivoreName);
    }

     /**
     * Stops the motors.
     */
    public void brake() {
        elevatorMotorOne.setVoltage(0);
        elevatorMotorTwo.setVoltage(0);
    }

    /**
     * Returns the average position of the elevator.
     * @param physical If the value returned should account for gear ratio.
     * @return The average position of the elevator motors.
     */
    public double getElevatorPosition(boolean physical) {
        double motorOnePosition = Units.rotationsToDegrees(elevatorMotorOne.getPosition().getValueAsDouble());
        double motorTwoPosition = Units.rotationsToDegrees(elevatorMotorTwo.getPosition().getValueAsDouble());
        double averageMotorPosition = (motorOnePosition + motorTwoPosition) / 2;
        if (physical){
            return averageMotorPosition / ElvConst.gearRatio;
        } else {
            return averageMotorPosition;
        }
    }

    /**
     * Calculates the speed of the elevator.
     * @param physicalSpeed Whether gear ratio should be considered.
     * @return Average speed of both motors.
     */
    public double getElevatorSpeed(boolean physicalSpeed) {
        double speedMotorOne = elevatorMotorOne.getVelocity().getValueAsDouble();
        double speedMotorTwo = elevatorMotorTwo.getVelocity().getValueAsDouble();
        if (physicalSpeed) {
            return ((speedMotorOne + speedMotorTwo) / 2) / ElvConst.gearRatio;
        } else {
            return (speedMotorOne + speedMotorTwo) / 2;
        }
    }

    /**
     * @param output A decimal between -1 and 1 (inclusive) that indicates the % output of the motor.
     * @return If the elevator is traveling within limits.
     */
    public boolean checkElevatorMovement(double output) {
        if (getElevatorPosition(false) <= ElvConst.elevatorLowerBound) {
            if (output >= 0) {
                return true;
            } else {
                return false;
            }
        } else if (getElevatorPosition(false) >= ElvConst.elevatorUpperBound) {
            if (output < 0) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }

    /**
     * Sets the speed of the Elevator motor to the given value.
     * @param output A decimal between -1 and 1 (inclusive) that indicates the % output of the motor.
     */
    public void setSpeed(double output) {
        if (checkElevatorMovement(output)) {
            elevatorMotorOne.setVoltage(output);
            elevatorMotorTwo.setVoltage(output);
        } else {
            brake();
        }
    }

    /**
     * Sets the position of the Elevator to the given value in Degrees.
     * @param newPosition The new position of the elevator.
     */
    public void setPosition(double newPosition) {
        elevatorMotorOne.setPosition(Units.rotationsToDegrees(newPosition));
        elevatorMotorTwo.setPosition(Units.rotationsToDegrees(newPosition));
    }

    /**
     * Sets the position of the Elevator to the given value in Rotations.
     * @param newPosition The desired position for the elevator.
     */
    public void setPositionInRotations(double newPosition) {
        elevatorMotorOne.setPosition(newPosition);
        elevatorMotorTwo.setPosition(newPosition);
    }
}