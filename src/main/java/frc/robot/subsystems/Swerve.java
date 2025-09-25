package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.constants. SwvConst;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    /**
     * Creates a new Swerve subsystem object using four SwerveModule objects (representing each swerve module of the physical robot) 
     * and a Pigeon2 object. In addition, PathPlanner's AutoBuilder and the SwerveDriveOdometry object are configured in this method
     * @return A new Swerve subsystem object
     */
    public Swerve() {
        if ( SwvConst.usesCANivore) {
            gyro = new Pigeon2( SwvConst.pigeonID,  SwvConst.swerveCANivoreName);
            gyro.getConfigurator().apply(new Pigeon2Configuration());
            gyro.setYaw(0); // The robot should move in the direction the front wheels face when the robot is first booted up

            /* SwerveModule Objects */
            mSwerveMods = new SwerveModule[] { 
                new SwerveModule(0, SwvConst.Mod0.constants, SwvConst.swerveCANivoreName),
                new SwerveModule(1, SwvConst.Mod1.constants, SwvConst.swerveCANivoreName),
                new SwerveModule(2, SwvConst.Mod2.constants, SwvConst.swerveCANivoreName),
                new SwerveModule(3, SwvConst.Mod3.constants, SwvConst.swerveCANivoreName)
            };
        } else {
            gyro = new Pigeon2(SwvConst.pigeonID);
            gyro.getConfigurator().apply(new Pigeon2Configuration());
            gyro.setYaw(0); // The robot should move in the direction the front wheels face when the robot is first booted up

            /* SwerveModule Objects */
            mSwerveMods = new SwerveModule[] { 
                new SwerveModule(0, SwvConst.Mod0.constants),
                new SwerveModule(1, SwvConst.Mod1.constants),
                new SwerveModule(2, SwvConst.Mod2.constants),
                new SwerveModule(3, SwvConst.Mod3.constants)
            };
        }

        /* Swerve Drive Odometry (or, in the future, perhaps a Swerve Pose Estimator)
         * A SwerveDriveOdometry object works by constantly tracking the angle of rotation (yaw) of the robot chassis, the current angle 
         * of rotation of each swerve module wheel, and the total distance travelled by each swerve module wheel
         * By tracking the values mentioned above and being given the location of each swerve module relative to each other, the relative 
         * location of the robot and the swerve module wheel states may be tracked
         * WARNING: Unless a pose estimator with vision-based correction is used, the robot location measurements can become inaccurate with 
         * repeated collisions, rugged-terrain driving, or simply continuous driving
         */
        swerveOdometry = new SwerveDriveOdometry(SwvConst.swerveKinematics, getGyroYaw(), getModulePositions()); // TODO: Swap this with a SwerveDrivePoseEstimator if using vision

        /* PathPlanner AutoBuilder Configuration */
        var alliance = DriverStation.getAlliance();

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            
            AutoBuilder.configure( // Configure AutoBuilder for PathPlanner; it must happen IN THE CONSTRUCTOR AT THE END OF THE METHOD DEFINITION
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                SwvConst.AutoSwerveConstants.pathPlannerConfig, // Other configuration values from the Constants class of this project
                config, // Load the RobotConfig from the GUI settings.
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    /**
     * Sets the speeds (voltages) of the angle (azimuth) and drive motors of all swerve modules based on the desired robot movement and 
     *  mode of travel
     * @param translation A Translation2d object representing the desired x and y component of the robot chassis's desired translational movement
     * @param rotation A double representing the angular velocity (radians per second) the robot chassis should rotate at
     * @param fieldRelative If the swerve drive should be moving in field-relative mode or not. Field-relative mode moves the robot relative to 
     * the robot's heading. Heading, NOT TO BE CONFUSED WITH THE CHASSIS'S ORIENTATION, is the direction the swerve drive considers "forward" when 
     * doing field-relative movement. Not choosing to use field-relative movement will result in robot-relative movement instead -- movement 
     * relative to the direction the robot chassis faces (essentially like how a typical car drives)
     * @param isOpenLoop If open loop mode should be used. Use open loop mode for teleoperated swerve drive and closed loop mode (having 
     * isOpenLoop = false) for autonomous swerve drive
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        /* Calculating the desired motor voltages for each swerve module... */
        SwerveModuleState[] swerveModuleStates =
            SwvConst.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds( // If the swerve drive is using field-relative controls, do the following...
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading() 
                                ) // ...otherwise, use robot-relative controls:
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                ); // NOTE: Please see the method documentation if you are unsure of what field- and robot-relative controls are

        /* Altering the calculated motor voltages such that the resulting wheel speeds will move the robot faster than its limit... */
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwvConst.maxSpeed); 

        /* Applying the calculated (and, if necessary, altered) desired motor voltages to each motor of each swerve module... */
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /**
     * Gets the speed and angle of rotation of each swerve module wheel as SwerveModuleState objects
     * @return A SwerveModuleState array that contains the SwerveModuleState objects for each swerve module in order of the swerve 
     * module numbers. A SwerveModuleState object contains the speed and angle of rotation of the swerve module the object represents
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * Gets the angle of rotation and total distance travelled of each swerve module wheel as SwerveModulePosition objects
     * @return A SwerveModulePosition array that contains the SwerveModulePosition objects for each swerve module in order of the swerve 
     * module numbers. A SwerveModulePosition object contains the angle of rotation and total distance travelled of the swerve module 
     * the object represents
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Gets the x and y components of ROBOT-RELATIVE translational movement along with the angular velocity of the robot chassis (radians 
     * per second) as a ChassisSpeeds object. Note that, here, x represents forward movement of the robot chassis and y represents 
     * left/right movement of the robot chassis
     * @return A ChassisSpeeds object that contains values representing the x and y components of ROBOT-RELATIVE translational movement 
     * along with the angular velocity of the robot chassis (radians per second) 
     */
    public ChassisSpeeds getChassisSpeeds() { // For PathPlanner
        ChassisSpeeds fieldRelativeSpeeds = SwvConst.swerveKinematics.toChassisSpeeds(getModuleStates());
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getGyroYaw());
        return robotRelativeSpeeds;
    }

    /**
     * Uses the previously-defined drive() method and a ChassisSpeeds object to perform ROBOT-RELATIVE driving
     * @param speeds A ChassisSpeeds object that contains the desired ROBOT-RELATIVE (robot chassis) translational movement and angular 
     * velocity of the robot chassis (radians per second)
     */
    public void driveRobotRelative(ChassisSpeeds speeds) { // For PathPlanner
        this.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, false);
    }

    public Pose2d getPose() { // For PathPlanner
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) { // For PathPlanner
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        /* All swerve drive odometries or swerve pose estimators MUST be updated periodically */
        swerveOdometry.update(getGyroYaw(), getModulePositions()); 

        /* Send debugging information to SmartDashboard (or, in the future, perhaps AdvantageScope) */
        for(SwerveModule mod : mSwerveMods){ // TODO: Change to use AdvantageScope's features for better visual feedback
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}