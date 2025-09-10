package frc.robot.subsystems;

import frc.robot.constants.MailboxConst;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.SparkFlex; //download stuff

public class Mailbox extends SubsystemBase {
    private final Timer coralTimer; //timer obj
    private final SparkFlex intakeVortex; //spark obj; speed go set yes eys
    private final DigitalInput coralSensor; //dig inp obj


    public Mailbox() { //mailbox constructgefisdisdh
        coralTimer = new Timer();
        coralTimer.start(); //begin
        intakeVortex = new SparkFlex(MailboxConst.mailboxMotorID, null);
        coralSensor = new DigitalInput(MailboxConst.mailboxChannelID); //mailbox constant stuff here
    }

    public void setSpeed(double voltage) {
        intakeVortex.setVoltage(-voltage); // add "-"" for going other way
    }

    public void brake() {
        intakeVortex.set(0);
    }

    /**
     * beam break sensor states false when the b
     * @return a boolean that does...
     */
    public boolean coralDetected() {
        return !coralSensor.get(); //beam break sensor states false when the beam is broken and true when the beam is unbroken so we inverse it
    }

    public boolean canScore() {
        if (coralDetected()) {
            coralTimer.reset();
        }
        return (coralTimer.get() < MailboxConst.scoringLeewayTime); //leeway: period of time that has been lost/wasted
    }
}