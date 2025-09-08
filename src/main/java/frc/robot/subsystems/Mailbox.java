package frc.robot.subsystems;

import frc.robot.constants.MailboxConst;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.SparkFlex; //download stuff

public class Mailbox extends SubsystemBase{
    private final Timer coralTimer; //timer obj
    private final SparkFlex intakeVortex; //spark obj; speed go set yes eys
    private final DigitalInput sensorForCoral; //dig inp obj


public Mailbox(){ //mailbox constructgefisdisdh
    coralTimer = new Timer();
    coralTimer.start(); //begin
    intakeVortex = new SparkFlex(0, null);
    sensorForCoral = new DigitalInput(0); //mailbox constant stuff here
}

public void setSpeed(double voltage){
    intakeVortex.setVoltage(voltage); // add "-"" for going other way
}

public void brake(){
    intakeVortex.set(0);
}

public boolean detectorForCoral(){
    return sensorForCoral.get(); //need ! (?)
}

public void scorerCoral(){
    if (detectorForCoral())
        coralTimer.reset();
    }
    /*else {
        return (coralTimer.get() <MailboxConstants.scoringLeewayTime)) //leeway: period of time that has been lost/wasted

    }*/


}