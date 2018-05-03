package org.usfirst.frc.team67.PathPlanning;

public class MotionProfile
{
	// States for motion profile control
	public static final Integer DISABLED = 0;
	public static final Integer STARTING = 1;
	public static final Integer PROFILING = 2;
	public static final Integer ENDING = 3;

	// deltaTime(s)
	public final Double deltaTime;

	// Velocity, Heading, DeltaTime
	public final double[][] LeftProfile;
	// Velocity, Heading, DeltaTime
	public final double[][] RightProfile;

	public MotionProfile(double dt, double[][] left, double[][] right)
	{
		deltaTime = dt;

		LeftProfile = left;
		RightProfile = right;
	}

}
