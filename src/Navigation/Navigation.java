package Navigation;

import java.util.ArrayList;
import java.util.Collections;
import Application.StartRobot;
import Odometer.Odometer;
import SensorData.USPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is in charge of all of the robots movements. It retrieves the
 * position information from the odometer and the lower ultrasonic sensor to
 * calculate its movements. Its main three methods allow the robot to turn to a
 * specific heading, travel to specific XY positions on the board & identify
 * wooden blocks and blue blocks based on height.
 * 
 * @author Sean Lawlor, Sebastian Andrade, Alexander Bratyshkin
 *
 */
public class Navigation {
	final static int FAST = 200, SLOW = 180, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 2, CM_ERR_2 = 1;

	// Board constants
	private static final int MAX_X_BOARD = -3;
	private static final int MIN_X_BOARD = 1;
	private static final int MAX_Y_BOARD = 3;
	private static final int MIN_Y_BOARD = 1;

	// ScanObject constants
	private static final int SCAN_TIME = 10;

	// Object avoidance constants
	private static final double MIN_DISTANCE = 6;
	private static final double EDGE_DISTANCE = 30;
	private static final double SAFETY_ANGLE = 50;

	// isWooden constants
	private static final double DELTA_DISTANCE = 9;
	private static final double SCAN_ANGLE = 20;
	private static final int DETECTION_ANGLE = 0;

	private static final int SLOW_ROTATE_SPEED = 50;
	private static final int FORWARD_SPEED = 200;
	private static final int WAYPOINT_BLOCKED_BW = 3;

	private static final int ROTATE_SPEED = 80;

	// Zone avoidance constants
	private static final int SAFE_DISTANCE = 15;
	private static final int ERROR_ZONE = 5;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private USPoller us, highUs;

	/**
	 * Creates a new Navigation object to control all the movements of the
	 * robot.
	 * 
	 * @param odo
	 *            System odometer
	 * @param us
	 *            Lower Ultrasonic Poller
	 * @param highUs
	 *            Higher Ultrasonic Poller
	 */
	public Navigation(Odometer odo, USPoller us, USPoller highUs) {
		this.odometer = odo;
		this.us = us;
		this.highUs = highUs;
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * This method sets up the speeds of the two wheels independently. If any of
	 * the speeds is negative, the speed will be set as positive and the
	 * corresponding motor will move backwards.
	 * 
	 * @param lSpd
	 *            Speed of left wheel
	 * @param rSpd
	 *            Speed of right wheel
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/**
	 * Sleeps the thread for time miliseconds.
	 * 
	 * @param time
	 *            in miliseconds
	 */
	public void waitMs(long time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// Do nothing, it should never happen
			e.printStackTrace();
		}
	}

	/**
	 * Stops the motors & floats them. This allow the motors to rotate freely
	 * while the robot is still running.
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/**
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading This version of the method exits with boolean false if see an
	 * obstacle.
	 * 
	 * @param x
	 *            desired X
	 * @param y
	 *            desired Y
	 * @return true if it reached the desired XY position, false otherwise.
	 */
	public boolean travelTo(double x, double y) {
		double minAng;

		// Calculate heading of desired XY point
		minAng = getMinAng(x, y);

		// Go to calculated heading
		this.turnTo(minAng, true);

		do {
			// Set speeds to go forward fast
			this.setSpeeds(FAST, FAST);

			// if lower ultrasonic poller sees an object within MIN_DISTANCE,
			// returns false
			if (us.getDistance() < MIN_DISTANCE) {
				return false;
			}
			// While far away of desired XY, keep going forward.
		} while (Math.abs(Math.abs(x) - Math.abs(odometer.getX())) > CM_ERR

				|| Math.abs(y - odometer.getY()) > CM_ERR);

		// Stop the robot when destination is reached.
		this.setSpeeds(0, 0);
		return true;
	}

	/**
	 * 
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading. When avoid = true, it will avoid objects based on the lower
	 * ultrasonic poller readings.
	 * 
	 * @param x
	 *            desired X
	 * @param y
	 *            desired Y
	 * @param avoid
	 *            activate objectAvoidance
	 */
	public void travelTo(double x, double y, boolean avoid) {

		double minAng;
		// double[] data = new double[4];
		boolean object = false;
		// double[] blockProperties = new double[2];
		boolean nearForbiddenZone = false;

		minAng = getMinAng(x, y);
		this.turnTo(minAng, true);

		while ((Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR)) {

			this.setSpeeds(FAST, FAST);

			// if object too close, object avoidance
			if (us.getDistance() < MIN_DISTANCE && avoid) {
				// avoid object
				avoidObject(true);
				// keep traveling to original destination
				travelTo(x, y, true);

			}

			// Detect if it's close zone to avoid
			if (((odometer.getX() + ERROR_ZONE) > StartRobot.LFZx || (odometer.getX() + ERROR_ZONE) > StartRobot.UFZx)
					&& ((odometer.getY() + ERROR_ZONE) > StartRobot.LFZy
							|| (odometer.getY() + ERROR_ZONE) > StartRobot.UFZy)) {
				nearForbiddenZone = true;
				break;
			}

		}

		this.setSpeeds(0, 0);

		// Zone avoidance
		if (nearForbiddenZone) {

			// Go backwards to guarantee a minimum distance from the forbidden
			// zone
			this.goForward(-ERROR_ZONE);

			// Load an array with all the coordinates of the forbidden zone
			double[][] points = new double[4][2];
			points[0][0] = StartRobot.LFZx;
			points[0][1] = StartRobot.LFZy;
			points[1][0] = StartRobot.UFZx;
			points[1][1] = StartRobot.LFZy;
			points[2][0] = StartRobot.LFZx;
			points[2][1] = StartRobot.UFZy;
			points[3][0] = StartRobot.UFZx;
			points[3][1] = StartRobot.UFZy;

			// Calculates coordinates of lower corner of forbidden zone
			double distanceLowerCorner1 = distance(x, y, points[0][0], points[0][1]); // Initialize
																						// //
																						// shortestDistance
			double distanceLowerCorner2 = distance(x, y, points[1][0], points[1][1]);

			// Calculates coordinates of Upper corner of forbidden zone
			double distanceUpperCorner1 = distance(x, y, points[2][0], points[2][1]);
			double distanceUpperCorner2 = distance(x, y, points[3][0], points[3][1]);

			ArrayList<Double> distances = new ArrayList<Double>();
			distances.add(distanceLowerCorner1);
			distances.add(distanceLowerCorner2);
			distances.add(distanceUpperCorner1);
			distances.add(distanceUpperCorner2);

			// Getting the shortest distance
			Collections.sort(distances);

			// If first lowerCorner is the closest one
			if (distances.get(0) == distanceLowerCorner1) {

				// If forbidden zone is closer to X axis
				if (Math.abs(odometer.getX() - points[0][0]) < Math.abs(odometer.getY() - points[0][1])) {
					// Move along the Y axis
					travelTo(points[0][0] - SAFE_DISTANCE, odometer.getY(), true);

					// Move along X axis
					travelTo(odometer.getX(), points[0][1] + SAFE_DISTANCE, true);

					// Go to final position
					travelTo(x, y, true);

				} else {
					// If forbidden zone is closer to Y axis

					// Move along the X axis
					travelTo(odometer.getX(), points[0][1] + SAFE_DISTANCE, true);

					// Move along the Y axis
					travelTo(points[0][0] - SAFE_DISTANCE, odometer.getY(), true);

					// Go to final position
					travelTo(x, y, true);
				}

			}

			// if second lower corner is closer
			if (distances.get(0) == distanceLowerCorner2) {

				// If forbidden zone is closer to X axis
				if (Math.abs(odometer.getX() - points[1][0]) < Math.abs(odometer.getY() - points[1][1])) {

					// Move along the Y axis
					travelTo(points[1][0] - SAFE_DISTANCE, odometer.getY(), true);

					// Move along the X axis
					travelTo(odometer.getX(), points[1][1] + SAFE_DISTANCE, true);

					// Go to final position
					travelTo(x, y, true);
				} else {

					// Move along the X axis
					travelTo(odometer.getX(), points[1][1] + SAFE_DISTANCE, true);

					// Move along the Y axis
					travelTo(points[1][0] - SAFE_DISTANCE, odometer.getY(), true);

					// Go to final position
					travelTo(x, y, true);
				}
			}

			// If first upper Corner is the closest one
			if (distances.get(0) == distanceUpperCorner1) {

				if (Math.abs(odometer.getX() - points[2][0]) < Math.abs(odometer.getY() - points[2][1])) {
					travelTo(points[2][0] - SAFE_DISTANCE, odometer.getY(), true);
					travelTo(odometer.getX(), points[2][1] + SAFE_DISTANCE, true);
					travelTo(x, y, true);
				} else {
					travelTo(odometer.getX(), points[2][1] + SAFE_DISTANCE, true);
					travelTo(points[2][0] - SAFE_DISTANCE, odometer.getY(), true);
					travelTo(x, y, true);
				}
			}

			if (distances.get(0) == distanceUpperCorner2) {
				// System.out.println("I am here 4");

				if (Math.abs(odometer.getX() - points[3][0]) < Math.abs(odometer.getY() - points[3][1])) {
					travelTo(points[3][0] - SAFE_DISTANCE, odometer.getY(), true);
					travelTo(odometer.getX(), points[3][1] + SAFE_DISTANCE, true);
					travelTo(x, y, true);
				} else {
					travelTo(odometer.getX(), points[3][1] + SAFE_DISTANCE, true);
					travelTo(points[3][0] - SAFE_DISTANCE, odometer.getY(), true);
					travelTo(x, y, true);
				}
			}

		}
	}

	/**
	 * When a wooden block is detected, the robot scans the edge of the block
	 * with the lower ultrasonic sensor. Once the edge is detected, the robot
	 * turns by SAFETY_ANGLE from that position, goes forward a given 10cm (20cm
	 * if full = true), turns to its original heading and moves
	 * 
	 * @param full:
	 *            when true, turns away from the object with a bigger angle
	 */
	public void avoidObject(boolean full) {

		// Stop robot
		this.setSpeeds(0, 0);

		// Record initial angle
		double initialAngle = odometer.getTheta();
		double angle;

		// Rotate on its axis
		this.setSpeeds(-SLOW, SLOW);

		while (true) {
			// Keep rotating until
			if (us.getDistance() > EDGE_DISTANCE) {
				angle = odometer.getTheta();
				break;
			}
		}
		this.setSpeeds(0, 0);

		if (full) {
			// Turn away from the object by a SAFETY_ANGLE
			turnTo(angle + SAFETY_ANGLE, true);

		} else {

			// Turn away from the object by a 60% of SAFETY_ANGLE
			turnTo(angle + 0.6 * SAFETY_ANGLE, true);
		}

		goForward(30);

		turnTo(initialAngle, true);
		goForward(15);

	}

	public double getMinAng(double x, double y) {
		double minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
		if (minAng < 0) {
			minAng += 360.0;
		}
		return minAng;
	}

	/**
	 * 
	 * TurnTo function which takes an angle and boolean as arguments The boolean
	 * controls whether or not to stop the motors when the turn is completed
	 * 
	 * @param angle
	 *            turn to this heading
	 * @param stop
	 *            if true, stop when reaching angle. If false, keep rotating.
	 */
	public void turnTo(double angle, boolean stop) {

		// Calculate difference between desired angle & current angle
		double error = angle - this.odometer.getTheta();

		//
		while (Math.abs(error) > DEG_ERR) {
			// Recalculate error every time loop runs
			error = angle - this.odometer.getTheta();

			// Choose smallest turn based on the value of the error
			if (error < -180.0) {
				this.setSpeeds(-SLOW, SLOW);
			} else if (error < 0.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else if (error > 180.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else {
				this.setSpeeds(-SLOW, SLOW);
			}
		}

		// If stop = true, stop when angle is reached.
		if (stop) {
			this.setSpeeds(0, 0);
		}
	}

	/**
	 * This method identifies the block and measures the distance between the
	 * object and the robot. To do this, the robot rotates (SCAN_ANGLE * 2)
	 * degrees recording distances with both ultrasonic sensors. This allows to
	 * detect the height of the object.
	 * 
	 * @return results An array of doubles where the first element identifies
	 *         the block type results[0] = 1 => Wooden block results[0] = 0 =>
	 *         Blue block
	 * 
	 *         The second element is the closest distance measured by the lower
	 *         ultrasonic poller results[1] = minLow
	 */
	public double[] isWooden() {
		double minLow, minHigh, initialAngle;
		double[] results = new double[2];

		// Save both sensors initial distances
		minLow = us.getDistance();
		minHigh = highUs.getDistance();
		initialAngle = odometer.getTheta();
		double bestAngle = initialAngle;

		// Turn to starting angle
		turnTo(initialAngle - SCAN_ANGLE, true);

		// Starts rotating
		setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

		// Scan until ending angle is reached
		while (odometer.getTheta() < initialAngle + SCAN_ANGLE) {
			// Rercord the smallest distance recorded by lower sensor & at which
			// heading it occurs
			if (us.getFilteredDistance() < minLow) {
				minLow = us.getFilteredDistance();
				bestAngle = odometer.getTheta();
			}

			// Records the smallest distance from the higher ultrasonic sensor
			if (highUs.getFilteredDistance() < minHigh) {
				minHigh = highUs.getFilteredDistance();
			}

			// Sleeps the thread
			try {
				Thread.sleep(SCAN_TIME / 2);
			} catch (InterruptedException e) {
			}
		}

		// Stop the robot
		stop();

		// Identify object
		if (minHigh < minLow + DELTA_DISTANCE) {
			results[0] = 1; // wooden
		} else {
			results[0] = 0; // blue block
		}

		// If blue block, rotate to the angle at which
		// the smallest distance was recorded
		if (results[0] == 0) {
			if (bestAngle < initialAngle) {
				turnTo(bestAngle - DETECTION_ANGLE, true);
			} else {
				turnTo(bestAngle + DETECTION_ANGLE, true);
			}
		}

		results[1] = minLow;
		return results;
	}

	/**
	 * Moves forward the given distance. This method rotates the motors by the
	 * calculated angle and does not return until the rotations or over.
	 * 
	 * @param distance
	 *            to move forward by
	 */
	public void goForward(double distance) {
		// Start the robot to move forward
		this.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

		leftMotor.rotate((int) ((180.0 * distance) / (Math.PI * 2.1)), true);
		rightMotor.rotate((int) ((180.0 * distance) / (Math.PI * 2.1)), false);
	}

	/**
	 * Stops both wheels.
	 */
	public void stop() {
		leftMotor.stop();
		rightMotor.stop();
	}

	/**
	 * @return leftMotor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	/**
	 * @param leftMotor
	 *            set leftMotor
	 */
	public void setLeftMotor(EV3LargeRegulatedMotor leftMotor) {
		this.leftMotor = leftMotor;
	}

	/**
	 * @return rightMotor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	/**
	 * @param rightMotor
	 *            set rightMotor
	 */
	public void setRightMotor(EV3LargeRegulatedMotor rightMotor) {
		this.rightMotor = rightMotor;
	}

	/** Compute the distance between two points (x1, y1) and (x2, y2) */
	public static double distance(double x1, double y1, double x2, double y2) {
		return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	}

}
