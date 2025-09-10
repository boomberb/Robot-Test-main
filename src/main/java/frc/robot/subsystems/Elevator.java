package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConst;

public class Elevator extends SubsystemBase {
    public TalonFX elevatorMotorOne;
    public TalonFX elevatorMotorTwo;

    public Elevator () {
        elevatorMotorOne = new TalonFX(ElevatorConst.elevatorMotorID);
        elevatorMotorTwo = new TalonFX(ElevatorConst.elevatorMotorID);
    }

    public void setSpeed (double output) {
        elevatorMotorOne.setVoltage(output);
        elevatorMotorTwo.setVoltage(output);
    }

    public void brake () {
        elevatorMotorOne.setVoltage(0);
        elevatorMotorTwo.setVoltage(0);
    }

    /* ah*/ public void setPosition(){

    }

    /* ah2 */ public void setPositionInRotations(){

    }
 
    /* ah3 */ public double getElevatorPosition(){
        return 0;
    }

    /* ah4 */ public double getElevatorSpeed(){
        return 0;
    }

    /* ah5 */ public double checkElevatorMovement(){
        return 0;
    }


}
