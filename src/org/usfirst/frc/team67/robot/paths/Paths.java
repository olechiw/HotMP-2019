package org.usfirst.frc.team67.robot.paths;

import org.usfirst.frc.team67.PathPlanning.MotionProfile;

public class Paths
{
	public static final MotionProfile Curve = new MotionProfile(SimpleCurve.deltaTime, SimpleCurve.Left,
			SimpleCurve.Right);
}
