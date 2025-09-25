package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.constants.SwvConst;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    /**
     * Stolen from Evan Wang.
     * @param s_Swerve
     * @param translationSup
     * @param strafeSup
     * @param rotationSup
     * @param robotCentricSup
     */
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get controller-input values from the DoubleSupplier objects and apply deadband values... */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), SwvConst.stickDeadband); // x-component of movement (FORWARD/BACKWARD)
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), SwvConst.stickDeadband); // y-component of movement (RIGHT/LEFT)
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), SwvConst.stickDeadband); // Rotation component of movement (CCW/CW)

        /* Use the Swerve object's drive method based on the obtained controller-input values... */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwvConst.maxSpeed), 
            rotationVal * SwvConst.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}