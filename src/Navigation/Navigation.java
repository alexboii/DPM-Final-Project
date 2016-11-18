package Navigation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import Application.Vector;
/*
 * File: Navigation.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Movement control class (turnTo, travelTo, flt, localize)
 */
import Odometer.Odometer;
import SensorData.USPoller;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 2.7;

	// ScanObject constants
	private static final double ANGLE_LIMIT = 80;
	private static final double MARGIN = 0.15;
	private static final double SCAN_DISTANCE = 5;
	private static final int SCAN_TIME = 30;
	private static final double US_OFFSET = 20;

	// Object avoidance constants
	private static final double MIN_DISTANCE = 10;
	private static final double SAFETY_RATIO = 1.8;
	private static final double SAFETY_RATIO_2 = 1.5;

	// isWooden constants
	private static final double MIN_WOODEN_SIZE = 16.5;

	// temp variable, debug purposes
	public static double[] data = new double[4];

	public static final int SLOW_ROTATE_SPEED = 30;

	public static final int ROTATE_SPEED = 50;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private USPoller us;

	public Navigation(Odometer odo, USPoller us) {
		this.odometer = odo;
		this.us = us;
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/*
	 * Functions to set the motor speeds jointly
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

	public void setSpeeds(int lSpd, int rSpd) {
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

	public double[] scanObject() {
		double[] results = new double[4];
		ArrayList<Vector> list_of_vectors = new ArrayList<Vector>();
		double initialDistance = us.getDistance();
		results[3] = initialDistance;
		data[3] = initialDistance;
		double initialAngle = odometer.getTheta();
		Vector vector = new Vector(us.getDistance(), odometer.getTheta());
		;
		Vector firstObjectEdge, secondObjectEdge;
		double alpha, beta;
		double leftLength, rightLength, length;
		double ratio;

		boolean wall = false;

		// First edge
		while ((!wall) && (us.getDistance() < Math
				.abs((initialDistance / Math.abs(Math.cos(Math.toRadians(odometer.getTheta()))))
						+ initialDistance * MARGIN))) {
			waitMs(SCAN_TIME);

			setSpeeds(-SLOW_ROTATE_SPEED, SLOW_ROTATE_SPEED);

			vector = new Vector(us.getDistance(), odometer.getTheta());
			list_of_vectors.add(vector);

			if (initialAngle - odometer.getTheta() > ANGLE_LIMIT) {
				wall = true;
				Sound.beep();
			}
		}

		if (list_of_vectors.size() != 0) {
		}
		// stop();

		firstObjectEdge = list_of_vectors.get(list_of_vectors.size() - 1);
		list_of_vectors.removeAll(list_of_vectors);

		turnTo(initialAngle, false);

		vector = new Vector(us.getDistance(), odometer.getTheta());

		setSpeeds(SLOW_ROTATE_SPEED, -SLOW_ROTATE_SPEED);

		// Second edge
		while ((!wall) && (us.getDistance() < Math
				.abs((initialDistance / Math.abs(Math.cos(Math.toRadians(odometer.getTheta()))))
						+ initialDistance * MARGIN))) {
			waitMs(SCAN_TIME);

			vector = new Vector(us.getDistance(), odometer.getTheta());
			list_of_vectors.add(vector);

			if (odometer.getTheta() - initialAngle > ANGLE_LIMIT) {
				// wall = true;
				// Sound.beep();
			}
		}

		stop();

		turnTo(initialAngle, true);

		secondObjectEdge = list_of_vectors.get(list_of_vectors.size() - 1);

		list_of_vectors.removeAll(list_of_vectors);

		alpha = initialAngle - firstObjectEdge.getAngle();
		beta = secondObjectEdge.getAngle() - initialAngle;

		leftLength = 2 * Math.abs(initialDistance * Math.tan(Math.toRadians(alpha)));
		rightLength = 2 * Math.abs(initialDistance * Math.tan(Math.toRadians(beta)));

		if (wall) {
			results[0] = -1;
			results[1] = -1;
			results[2] = -1;
		} else {

			length = rightLength + leftLength;
			ratio = length;

			// length= ((int)((length / 5) + 0.5 )) * 5; ;
			ratio = (length) / ratio;

			if (length > 20)
				length = 20;

			results[0] = length;
			results[1] = rightLength * ratio;
			results[2] = leftLength * ratio;
		}
		data = results; // static variable contains last set of results for
						// print&debug
		return results;
	}

	public void waitMs(long time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// Do nothing, it should never happen
			e.printStackTrace();
		}
	}

	/*
	 * Float the two motors jointly
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading
	 */
	public void travelTo(double x, double y) {
		double minAng;
		double[] data = new double[4];
		boolean object = false;

		while ((Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) && (!object)) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);

			if (minAng < 0)
				minAng += 360.0;

			this.turnTo(minAng, false);

			// if(x < 0 && y < 0){
			// this.setSpeeds(-FAST, -FAST);
			// }
			// else{
			// this.setSpeeds(FAST, FAST);
			// }
			//

			this.setSpeeds(FAST, FAST);

			// if object too close, object avoidance

			if (us.getDistance() < MIN_DISTANCE) {

				object = true;
				this.setSpeeds(0, 0);
				data = scanObject();
				turnTo(odometer.getTheta() + 55, true);
				goForward(Math.abs(data[0] * SAFETY_RATIO));
				travelTo(x, y);
			}

		}
		if (!object) {
			this.setSpeeds(0, 0);
		}

	}

	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean
	 * controls whether or not to stop the motors when the turn is completed
	 */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getTheta();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odometer.getTheta();

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

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}

	public boolean isWooden() {

		double[] data = new double[4];
		data = scanObject();
		if (data[0] > MIN_WOODEN_SIZE) {
			return true;
		} else {
			return false;
		}
	}

	// debug purposes
	public double[] getData() {
		return data;
	}

	public void goForward(double distance) {
		this.travelTo(odometer.getX() + Math.cos(Math.toRadians(this.odometer.getTheta())) * distance,
				odometer.getY() + Math.sin(Math.toRadians(this.odometer.getTheta())) * distance);

	}

	public void stop() {
		leftMotor.stop();
		rightMotor.stop();
	}

	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	public void setLeftMotor(EV3LargeRegulatedMotor leftMotor) {
		this.leftMotor = leftMotor;
	}

	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	public void setRightMotor(EV3LargeRegulatedMotor rightMotor) {
		this.rightMotor = rightMotor;
	}
}
