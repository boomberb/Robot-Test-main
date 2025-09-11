package frc.robot.subsystems;

import frc.robot.constants.MbxConst;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.SparkFlex; //download stuff
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**Represents the Mailbox subsystem and its functions.*/
public class Mailbox extends SubsystemBase {
    
    private final Timer coralTimer; //timer obj
    private final SparkFlex intakeVortex; //spark obj; speed go set yes eys
    private final DigitalInput coralSensor; //dig inp obj

    public Mailbox() { //mailbox constructgefisdisdh
        coralTimer = new Timer();
        coralTimer.start(); //begin
        intakeVortex = new SparkFlex(MbxConst.sparkMotorID, MotorType.kBrushless); //idk motor type
        coralSensor = new DigitalInput(MbxConst.channelID);
    }

    /**
     * Sets the speed of the Mailbox motor to the given value.
     * @param output A decimal between -1 and 1 (inclusive) that indicates the % output of the motor.
     */ 
    public void setSpeed(double output) {
        intakeVortex.setVoltage(-output * MbxConst.maxSpeed); //by default the motor spins counterclockwise so it is inversed
    }

    /**
     * Stops the motor.
     */
    public void brake() {
        intakeVortex.set(0);
    }

    /**
     * Receives input from the beam break sensor.
     * @return True when the beam is broken.
     */
    public boolean isCoralDetected() {
        return !coralSensor.get(); //by default the sensor states true when beam is not broken so this is inversed to make more sense
    }

    /**
     * Calculates if the gamepiece can be scored.
     * @return True if the time elapsed has not exceeded the leeway time.
     */
    public boolean canScore() {
        if (isCoralDetected()) {
            coralTimer.reset();
        }
        return (coralTimer.get() < MbxConst.scoringLeewayTime); //leeway: period of time that has been lost/wasted
    }
}