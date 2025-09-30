package frc.robot.constants;


/**
 * Constants used for the Elevator in subsystems and commands.
 */
public final class ElvConst {
    // IDs
    public static final int motorID = 0;

    // Limitations
    public static final double gearRatio = 1;
    public static final double lowerBound = 0;
    public static final double upperBound = 0;

    // Initialization
    public static final double startingHeight = 0;
    public static final double heightOne = 10;
    public static final double heightTwo = 20; 
    public static final double heightThree = 30;

    // Proportional Integral Derivative
    public static final double kP = 10;
    public static final double kI = 2.5;
    public static final double kD = 0;
    public static final double tolerance = 0;

    // Motion Profiling
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;

    // Feedforward
    public static final double setpointVelocity = 0;
}
