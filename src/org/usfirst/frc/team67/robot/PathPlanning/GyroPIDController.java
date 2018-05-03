package org.usfirst.frc.team67.robot.PathPlanning;

public class GyroPIDController
{
	private final double p, i, d;

	private double integral = 0;
	private double previousError = 0;

	/*
	 * Does not current attempt to use a feedforward for the turn pid
	 */
	public GyroPIDController(double _p, double _i, double _d)
	{
		p = _p;
		i = _i;
		d = _d;
	}

	// Jacis Nonsense angle corrector
	public static double correctedAngle(double angle_degrees)
	{
		while (angle_degrees >= 180.0)
			angle_degrees -= 360.0;
		while (angle_degrees < -180.0)
			angle_degrees += 360.0;
		return angle_degrees;
	}

	public double control(double targetAngle, double currentAngle, double dt)
	{
		double error = (correctedAngle(targetAngle) - correctedAngle(currentAngle));

		integral += (error * dt);
		double derivative = (error - previousError) / dt;

		previousError = error;

		return (p * error) + (i * integral) + (d * derivative);
	}

	public void reset()
	{
		integral = 0;
		previousError = 0;
	}
}
