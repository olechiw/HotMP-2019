/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team67.robot;

import org.usfirst.frc.team67.PathPlanning.PathFollower;
import org.usfirst.frc.team67.robot.paths.Paths;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot
{
	private static final int TIMEOUT = 10;

	private TalonSRX talonLeft;
	private TalonSRX talonRight;
	private PigeonIMU pigeon;
	private Joystick joy;

	private PathFollower pathFollower;

	@Override
	public void robotInit()
	{
		talonLeft = new TalonSRX(Constants.DRIVE_LEFT);
		talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT);
		talonRight = new TalonSRX(Constants.DRIVE_RIGHT);
		talonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, TIMEOUT);
		talonRight.setSensorPhase(true);

		pigeon = new PigeonIMU(Constants.GYRO);

		pathFollower = new PathFollower(talonLeft, talonRight, pigeon);
		pathFollower.SetConfig(Constants.PathFollowerConfig);

		joy = new Joystick(0);
	}

	@Override
	public void autonomousInit()
	{
	}

	@Override
	public void autonomousPeriodic()
	{
		pathFollower.SetPath(Paths.Curve);
		pathFollower.StartPath();
	}

	@Override
	public void teleopPeriodic()
	{
	}

	@Override
	public void testPeriodic()
	{
	}
}
