package org.usfirst.frc.team67.robot.PathPlanning;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Notifier;

public class HOTMotionController
{
	// Notifier for the closed loops
	private final Notifier controlNotifier;

	private final GyroPIDController gyroController;

	private final PigeonIMU pigeon;
	private final TalonSRX leftTalon;
	private final TalonSRX rightTalon;

	/*
	 * State for the closed loops
	 */
	private ControlMode state = ControlMode.Disabled;

	private Integer mpIndex = 0;
	private Integer mpState = MotionProfile.DISABLED;

	public HOTMotionController(TalonSRX left, TalonSRX right, PigeonIMU _pigeon)
	{
		this(left, right, _pigeon, MotionProfile.deltaTime);
	}

	public HOTMotionController(TalonSRX left, TalonSRX right, PigeonIMU _pigeon, double controlInterval)
	{
		leftTalon = left;
		rightTalon = right;
		pigeon = _pigeon;

		gyroController = new GyroPIDController(MotionProfile.TURN_P, MotionProfile.TURN_I, MotionProfile.TURN_D);

		/*
		 * Iffy about this part with the synchronized control function because that may
		 * delay the process of the motion profile
		 */
		controlNotifier = new Notifier(this::Control);
		controlNotifier.startPeriodic(controlInterval);
	}

	private synchronized void Control()
	{
		if (state.equals(ControlMode.MotionProfile))
		{
			ControlMP();
		} else if (state.equals(ControlMode.PercentOutput))
		{
			ControlArcade();
		}
	}

	private void ControlArcade()
	{
		// TODO Auto-generated method stub

	}

	private void ControlMP()
	{
		if (mpState == MotionProfile.ENDING)
		{
			leftTalon.set(ControlMode.Disabled, 0);
			rightTalon.set(ControlMode.Disabled, 0);
			gyroController.reset();
			mpIndex = 0;
			mpState = MotionProfile.DISABLED;
		} else if (mpState == MotionProfile.STARTING)
		{
			/*
			 * Config PID (80 ms timeout for all)
			 */
			leftTalon.config_kP(0, MotionProfile.V_L_P, 80);
			leftTalon.config_kI(0, MotionProfile.V_L_I, 80);
			leftTalon.config_kD(0, MotionProfile.V_L_D, 80);
			leftTalon.config_kF(0, MotionProfile.V_L_F, 80);

			rightTalon.config_kP(0, MotionProfile.V_R_P, 80);
			rightTalon.config_kI(0, MotionProfile.V_R_I, 80);
			rightTalon.config_kD(0, MotionProfile.V_R_D, 80);
			rightTalon.config_kF(0, MotionProfile.V_R_F, 80);

			/*
			 * Zero encoders
			 */
			leftTalon.setSelectedSensorPosition(0, 0, 80);
			rightTalon.setSelectedSensorPosition(0, 0, 80);

			/*
			 * Config error
			 */
			leftTalon.configAllowableClosedloopError(0, MotionProfile.allowedClosedLoopError, 80);
			rightTalon.configAllowableClosedloopError(0, MotionProfile.allowedClosedLoopError, 80);

			/*
			 * Zero gyro
			 */
			pigeon.setYaw(0, 80);

			mpState = MotionProfile.PROFILING;
		} else if (mpState == MotionProfile.PROFILING)
		{
			/*
			 * Check for end
			 */
			if (mpIndex > MotionProfile.LeftProfile.length)
			{
				mpState = MotionProfile.ENDING;
			}

			double velocityLeft = MotionProfile.LeftProfile[mpIndex][0];
			double velocityRight = MotionProfile.RightProfile[mpIndex][0];
			double heading = MotionProfile.LeftProfile[mpIndex][1];

			// Get heading correction
			double[] ypr = new double[3];
			pigeon.getYawPitchRoll(ypr);

			/*
			 * Currently only adjusts for velocity with leftmost drive. TODO: CHECK POSITIVE
			 * + NEGATIVE SIGNS FOR GYRO/FEED FORWARD
			 */
			double turnPIDResult = gyroController.control(heading, ypr[0], MotionProfile.deltaTime);
			double vDiff = (MotionProfile.TURN_F * turnPIDResult) / 2;

			leftTalon.set(ControlMode.Velocity, velocityLeft + vDiff);
			rightTalon.set(ControlMode.Velocity, velocityRight - vDiff);

			mpIndex++;
		}
	}

	/*
	 * State functions
	 */
	public synchronized ControlMode GetState()
	{
		return state;
	}

	public synchronized Integer GetMPState()
	{
		return mpState;
	}
}
