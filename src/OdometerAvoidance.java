/*
 * Odometer.java
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class OdometerAvoidance extends Thread {
	// robot position
	private double x, y, theta;
	private int leftMotorTachoCount, rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;
	private static final int TIMES_TWO = 2;
	private static final double HALF = 0.5;
	private static final int FULL_CIRCLE = 360;

	public static int lastTachoL; // Tacho L at last sample
	public static int lastTachoR; // Tacho R at last sample
	public static int nowTachoL; // Current tacho L
	public static int nowTachoR; // Current tacho R
	public double track;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public OdometerAvoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double track) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		this.track = track;
		lock = new Object();

		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();

		lastTachoL = 0;
		lastTachoL = 0;
		nowTachoL = 0;
		nowTachoR = 0;

	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {

			double leftDistance, rightDistance, changeDistance, changeTheta, dx, dy;

			updateStart = System.currentTimeMillis();

			nowTachoL = leftMotor.getTachoCount();
			nowTachoR = rightMotor.getTachoCount();

			// COMPUTE DISPLACEMENT OF BOTH WHEELS
			leftDistance = (TIMES_TWO * Math.PI * Lab5.WHEEL_RADIUS * (nowTachoL - lastTachoL)) / FULL_CIRCLE;
			rightDistance = (TIMES_TWO * Math.PI * Lab5.WHEEL_RADIUS * (nowTachoR - lastTachoR)) / FULL_CIRCLE;

			// SAVE TACHO COUNTS
			lastTachoL = nowTachoL;
			lastTachoR = nowTachoR;

			// COMPUTE THE CHANGE OF DISTANCE AND CHANGE IN THETA
			changeDistance = HALF * (leftDistance + rightDistance);
			changeTheta = (leftDistance - rightDistance) / track;

			synchronized (lock) {

				// COMPUTE X AND Y DISPLACEMENTS
				dx = changeDistance * Math.sin(theta);
				dy = changeDistance * Math.cos(theta);

				// COMPUTE CHANGE IN THETA
				theta += changeTheta;

				// UPDATE X AND Y
				x += dx;
				y += dy;

				// WRAP THE ANGLE
				if (theta < 0) {
					theta += 2 * Math.PI;
				}
				if (theta > (2 * Math.PI)) {
					theta -= 2 * Math.PI;
				}
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount
	 *            the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount
	 *            the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;
		}
	}
}