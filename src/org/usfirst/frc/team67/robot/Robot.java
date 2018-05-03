/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team67.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot
{
	private static final int LEFT = 1;
	private static final int RIGHT = 2;
	private static final int GYRO = 3;

	TalonSRX talonLeft;
	TalonSRX talonRight;

	@Override
	public void robotInit()
	{
		talonLeft = new TalonSRX(LEFT);
		talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 80);
		talonRight = new TalonSRX(RIGHT);
		talonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 80);
		talonRight.setSensorPhase(true);
	}

	@Override
	public void autonomousInit()
	{
	}

	@Override
	public void autonomousPeriodic()
	{
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
