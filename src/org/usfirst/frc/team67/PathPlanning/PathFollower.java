package org.usfirst.frc.team67.PathPlanning;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Notifier;

public class PathFollower
{
	// Notifier for the closed loops
	private final Notifier controlNotifier;

	private GyroPIDController gyroController;

	private final PigeonIMU pigeon;
	private final TalonSRX leftTalon;
	private final TalonSRX rightTalon;

	/*
	 * State for the closed loops
	 */
	private Integer mpIndex = 0;
	private Integer mpState = MotionProfile.DISABLED;
	private MotionProfile currentMP;

	private Config config;
	private Integer allowedClosedLoopError = 10;
	private int timeout = 10;

	/*
	 * Constructor with the configuration of PIDF provided
	 */
	public PathFollower(TalonSRX left, TalonSRX right, PigeonIMU _pigeon, Config _config)
	{
		this(left, right, _pigeon);
		config = _config;
	}

	/*
	 * Constructor with no config, one will need to be provided before anything can
	 * function
	 */
	public PathFollower(TalonSRX left, TalonSRX right, PigeonIMU _pigeon)
	{
		leftTalon = left;
		rightTalon = right;
		pigeon = _pigeon;

		/*
		 * Iffy about this part with the synchronized control function because that may
		 * delay the process of the motion profile
		 */
		controlNotifier = new Notifier(this::ControlMP);
	}

	/*
	 * Start following the path will all feedback devices
	 */
	public synchronized void StartPath()
	{
		if (currentMP != null && mpState == MotionProfile.DISABLED)
		{
			mpState = MotionProfile.STARTING;
			controlNotifier.startPeriodic(currentMP.deltaTime);
		}
	}

	/*
	 * Stop following the path will all devices
	 */
	public synchronized void StopPath()
	{
		mpState = MotionProfile.ENDING;
	}

	/*
	 * Control the state of the profile whether it is running, just started, or is
	 * scheduled to end
	 */
	private synchronized void ControlMP()
	{
		/*
		 * Assert integrity of all neccesary objects
		 */
		if (leftTalon == null || rightTalon == null || config == null || currentMP == null)
			return;

		if (mpState == MotionProfile.ENDING)
		{
			leftTalon.set(ControlMode.Disabled, 0);
			rightTalon.set(ControlMode.Disabled, 0);
			gyroController.reset();
			mpIndex = 0;
			mpState = MotionProfile.DISABLED;
			controlNotifier.stop();
		} else if (mpState == MotionProfile.STARTING)
		{
			if (config == null)
				return;

			/*
			 * Config PID (timeout ms timeout for all)
			 */
			leftTalon.config_kP(0, config.V_L_P, timeout);
			leftTalon.config_kI(0, config.V_L_I, timeout);
			leftTalon.config_kD(0, config.V_L_D, timeout);
			leftTalon.config_kF(0, config.V_L_F, timeout);

			rightTalon.config_kP(0, config.V_R_P, timeout);
			rightTalon.config_kI(0, config.V_R_I, timeout);
			rightTalon.config_kD(0, config.V_R_D, timeout);
			rightTalon.config_kF(0, config.V_R_F, timeout);

			/*
			 * Zero encoders
			 */
			leftTalon.setSelectedSensorPosition(0, 0, timeout);
			rightTalon.setSelectedSensorPosition(0, 0, timeout);

			/*
			 * Config error
			 */
			leftTalon.configAllowableClosedloopError(0, allowedClosedLoopError, timeout);
			rightTalon.configAllowableClosedloopError(0, allowedClosedLoopError, timeout);

			/*
			 * Zero gyro
			 */
			pigeon.setYaw(0, timeout);

			mpState = MotionProfile.PROFILING;
		} else if (mpState == MotionProfile.PROFILING)
		{
			/*
			 * Check for end (last point already served)
			 */
			if (mpIndex > currentMP.LeftProfile.length)
			{
				mpState = MotionProfile.ENDING;
			}

			double velocityLeft = currentMP.LeftProfile[mpIndex][0];
			double velocityRight = currentMP.RightProfile[mpIndex][0];
			double heading = currentMP.LeftProfile[mpIndex][1];

			// Get heading correction
			double[] ypr = new double[3];
			pigeon.getYawPitchRoll(ypr);

			/*
			 * Currently only adjusts for velocity with leftmost drive. TODO: CHECK POSITIVE
			 * + NEGATIVE SIGNS FOR GYRO/FEED FORWARD, CHECK UNITS
			 */
			double turnPIDResult = gyroController.control(heading, ypr[0], currentMP.deltaTime);
			double vDiff = (config.TURN_F * turnPIDResult) / 2;

			leftTalon.set(ControlMode.Velocity, velocityLeft + vDiff);
			rightTalon.set(ControlMode.Velocity, velocityRight - vDiff);

			mpIndex++;
		}
	}

	/*
	 * Set the motion profile path
	 */
	public synchronized void SetPath(MotionProfile path)
	{
		if (mpState == MotionProfile.DISABLED)
			currentMP = path;
	}

	/*
	 * Set the talon allowed closed loop error (probably not even neccesary)
	 */
	public synchronized void SetAllowedClosedLoopError(int error)
	{
		allowedClosedLoopError = error;
	}

	/*
	 * Set the PIDF config
	 */
	public synchronized void SetConfig(Config _config)
	{
		config = _config;
	}

	/*
	 * Get the current MP State (Starting, Profiling, Ending, Disabled)
	 */
	public synchronized Integer GetState()
	{
		return mpState;
	}

	/*
	 * Configuration class for PIDF
	 */
	public static class Config
	{
		// pidf for the talon's speed controller left side
		public final Double V_L_P;
		public final Double V_L_I;
		public final Double V_L_D;
		public final Double V_L_F;
		// pidf for the talon's speed controller right side
		public final Double V_R_P;
		public final Double V_R_I;
		public final Double V_R_D;
		public final Double V_R_F;

		// pidf for the custom gyro turn controller
		public final Double TURN_P;
		public final Double TURN_I;
		public final Double TURN_D;
		public final Double TURN_F;

		public Config(double kPl, double kIl, double kDl, double kFl, double kPr, double kIr, double kDr, double kFr,
				double kPt, double kIt, double kDt, double kFt)
		{
			V_L_P = kPl;
			V_L_I = kIl;
			V_L_D = kDl;
			V_L_F = kFl;

			V_R_P = kPl;
			V_R_I = kIl;
			V_R_D = kDl;
			V_R_F = kFl;

			TURN_P = kPt;
			TURN_I = kIt;
			TURN_D = kDt;
			TURN_F = kFt;
		}
	}
}
