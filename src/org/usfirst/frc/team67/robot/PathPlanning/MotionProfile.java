package org.usfirst.frc.team67.robot.PathPlanning;

public class MotionProfile
{
	// States for motion profile control
	public static final Integer DISABLED = 0;
	public static final Integer STARTING = 1;
	public static final Integer PROFILING = 2;
	public static final Integer ENDING = 3;

	// pidf for the talon's speed controller left side
	public static final double V_L_P = 0.0;
	public static final double V_L_I = 0.0;
	public static final double V_L_D = 0.0;
	public static final double V_L_F = 0.0;
	// pidf for the talon's speed controller right side
	public static final double V_R_P = 0.0;
	public static final double V_R_I = 0.0;
	public static final double V_R_D = 0.0;
	public static final double V_R_F = 0.0;

	// pidf for the custom gyro turn controller
	public static final double TURN_P = 0.0;
	public static final double TURN_I = 0.0;
	public static final double TURN_D = 0.0;
	public static final double TURN_F = 0.0;

	// deltaTime(s)
	public static final double deltaTime = .005;

	// allowable error in distance units
	public static final Integer allowedClosedLoopError = 0;

	// Velocity, Heading, DeltaTime
	public static final double[][] LeftProfile =
	{
			{ 0.0, 0.0, 0.0 } };
	// Velocity, Heading, DeltaTime
	public static final double[][] RightProfile =
	{
			{ 0.0, 0.0, 0.0 } };

}
