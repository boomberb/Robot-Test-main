package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElvConst;

/**
 * Represents the Elevator subsystem and its possible functions.
 */
public class Elevator extends SubsystemBase {
    public TalonFX elevatorMotorOne;
    public TalonFX elevatorMotorTwo;

    public Elevator() {
        elevatorMotorOne = new TalonFX(ElvConst.elevatorMotorID);
        elevatorMotorTwo = new TalonFX(ElvConst.elevatorMotorID);
    }

    /**
     * Sets the speed of the Elevator motor to the given value.
     * @param output A decimal between -1 and 1 (inclusive) that indicates the % output of the motor.
     */
    public void setSpeed(double output) {
        elevatorMotorOne.setVoltage(output);
        elevatorMotorTwo.setVoltage(output);
    }

    /** [WIP]
     * Stops the motor.
     */
    public void brake() {
        elevatorMotorOne.setVoltage(0);
        elevatorMotorTwo.setVoltage(0);
    }

    /** [WIP]
     * Sets the position of the Elevator to the given value.
     */
    public void setPosition() {

    }

    /** [WIP]
     * Sets the position of the Elevator to the given value in Rotations.
     */
    public void setPositionInRotations() {

    }

    /** [WIP]
     * @param position
     * @return
     */
    public double getElevatorPosition(boolean position) {
        double motorOnePosition = Units.rotationsToDegrees(elevatorMotorOne.getPosition().getValueAsDouble());
        double motorTwoPosition = Units.rotationsToDegrees(elevatorMotorTwo.getPosition().getValueAsDouble());
        double averageMotorPosition = (motorOnePosition + motorTwoPosition) / 2;
        if (position){
            //return averageMotorPosition --> divide by gear ratio in the constants file
        }
        else {
            return averageMotorPosition;
        }
        return 0;
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
            return ((speedMotorOne + speedMotorTwo) / 2) / ElvConst.elevatorGearRatio;
        } else {
            return (speedMotorOne + speedMotorTwo) / 2;
        }
    }

    public double checkElevatorMovement(double newSpeed) {
        return 0;
    }


}
