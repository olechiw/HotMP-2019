package org.usfirst.frc.team67.robot;

import org.usfirst.frc.team67.PathPlanning.PathFollower.Config;

public class Constants
{
	/*
	 * Left-Right-Gyro PIDF
	 */
	static final double leftP = 0.0, leftI = 0.0, leftD = 0.0, leftF = 0.0;
	static final double rightP = 0.0, rightI = 0.0, rightD = 0.0, rightF = 0.0;
	static final double turnP = 0.0, turnI = 0.0, turnD = 0.0, turnF = 0.0;

	/*
	 * Tank path follower config (4xPIDF in order of Left-Right-Turn)
	 */
	public static final Config PathFollowerConfig = new Config(leftP, leftI, leftD, leftF, rightP, rightI, rightD,
			rightF, turnP, turnI, turnD, turnF);

	/*
	 * CAN Bus IDs
	 */
	public static final int DRIVE_LEFT = 1;
	public static final int DRIVE_RIGHT = 2;
	public static final int GYRO = 3;
}
