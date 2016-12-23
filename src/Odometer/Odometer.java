package Odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

/**
 *
 * 
 * Class which controls the odometer for the robot
 * 
 * Odometer defines cooridinate system in an XY plane. 
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 * @author Sean Lawlor 
 * 
 */

public class Odometer implements TimerListener {

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private boolean rotating = false;
	public static boolean needCalibration = false;
	public static int zone = 0, lastZone = 0;
	private double[] oldDH, dDH;

	/**
	 * @param leftMotor
	 *            of the system
	 * @param rightMotor
	 *            of the system
	 * @param INTERVAL
	 *            custom value for the sleeping time
	 * @param autostart
	 *            true = it starts automatically
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL,
			boolean autostart) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.rightRadius = 2.1;
		this.leftRadius = 2.1;
		this.width = 18.5;

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
	 * Stops the timeListener
	 */
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}

	/**
	 * Starts the TimeListener
	 */
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}

	/**
	 * Calculates the displacement and the heading to go to a specific XY point
	 * on the board
	 * 
	 * @param data
	 *            Array of doubles containing the desire XY position
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}

	/**
	 * Recompute the odometer values using the displacement and heading changes
	 * 
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

		//// correction parameters

		zone = (int) (Math.abs(x) / (3 * 30.48));
		zone += 10 * (int) (y / (3 * 30.48));

		if (zone != lastZone) {
			needCalibration = true;
		}
		lastZone = zone;
	}

	/**
	 * @param status
	 *            true = odometer needs calibration
	 */
	public void setCalibration(boolean status) {
		needCalibration = status;
	}

	/**
	 * @return need of calibration of the odometer
	 */
	public boolean needsCalibration() {
		return needCalibration;
	}

	/**
	 * @return x value of the odometer
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * @return y value
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * @return theta
	 */
	public double getTheta() {
		synchronized (this) {
			return theta;
		}
	}

	/**
	 * @param theta
	 *            corrected
	 */
	public void setTheta(double theta) {
		synchronized (this) {
			this.theta = theta;
		}
	}

	/**
	 * @param y
	 *            corrected
	 */
	public void setY(double y) {
		synchronized (this) {
			this.y = y;
		}
	}

	/**
	 * @param x
	 *            corrected
	 */
	public void setX(double x) {
		synchronized (this) {
			this.x = x;
		}
	}

	// set x,y,theta
	/**
	 * @param position
	 *            array of doubles containing position[0] = x value position[1]
	 *            = y value position[2] = theta value
	 * 
	 * @param update
	 *            array of booleans specifying which values of the odometer are
	 *            to be updated update[0] = update X update[1] = update Y
	 *            update[2] = update theta
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
	 * @param position
	 *            array of doubles with the three values of the odometer
	 */
	public void setPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**
	 * @return array of doubles containing the three values of the odometer
	 */
	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}

	// accessors to motors
	/**
	 * @return an array of EV3LargeRegulatedMotors containing both the motors in
	 *         charge of the wheels of the robot
	 */
	public EV3LargeRegulatedMotor[] getMotors() {
		return new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor };
	}

	/**
	 * @return leftMotor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}

	/**
	 * @return rightMotor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/**
	 * @param angle
	 *            to fix
	 * @return angle wrapped around 360
	 */
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	/**
	 * @param rot
	 *            true if robot is currently rotating
	 */
	public void setRotating(boolean rot) {
		rotating = rot;
	}

	/**
	 * @return true if robot is rotating
	 */
	public boolean isRotating() {
		return rotating;
	}

	/**
	 * @param a
	 *            first angle
	 * @param b
	 *            second angle
	 * @return the minimum angle the robot needs to rotate to reach the desired
	 *         heading
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
}
