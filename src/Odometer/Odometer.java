package Odometer;

import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class will calculate the robot’s X and Y coordinates and angle from its
 * location relative to the starting point.
 * 
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 * @author Sean Lawlor ECSE 211 - Design Principles and Methods, Head TA Fall
 *         2011 Ported to EV3 by: Francois Ouellet Delorme Fall 2015
 *
 */
public class Odometer implements TimerListener {

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;

	/**
	 * Constructor 
	 * @param leftMotor
	 * @param rightMotor
	 * @param INTERVAL
	 * @param autostart
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL,
			boolean autostart) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// default values, modify for your robot
		this.rightRadius = 2.1;
		this.leftRadius = 2.1;
		this.width = 15.5;

		this.x = 0.0;
		this.y = 0.0;
		this.theta = 90.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else
			this.timer = null;
	}

	
	/**
	 * Stops the time listener
	 */
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}

	/**
	 * Starts the time listener
	 */
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}


	/**
	 * Calculates displacement and heading as title suggests
	 * @param left and right motors' data 
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}


	/**
	  * {@inheritDoc}
	  */
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	/**
	 * 
	 * @return X value
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * @return Y value
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * @return Theta value
	 */
	public double getTheta() {
		synchronized (this) {
			return theta;
		}
	}

	/**
	 * Set X, Y and Theta of Odometer 
	 * @param position X, Y and Theta 
	 * @param update
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * @param position X, Y and Theta of Odometer 
	 */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**
	 * @return position X, Y and Theta of Odometer 
	 */
	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}

	/**
	 * @return Both Motors
	 */
	public EV3LargeRegulatedMotor[] getMotors() {
		return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };
	}

	/**
	 * @return Left Motor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}

	/**
	 * @return Right Motors
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/**
	 * Wrap around the angle
	 * @param angle 
	 * @return Angle wrapped around 360
	 */
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

}
