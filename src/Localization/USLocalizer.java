package Localization;

import Navigation.Navigation;
import Odometer.Odometer;
import SensorData.USPoller;
import lejos.robotics.SampleProvider;

/**
 * This class contains all the necessary methods to perform ultrasonic localization by recording the angles
 * of the odometer at which the lower ultrasonic registered a reading smaller than DISTANCE_FROM_WALL.
 *  It calibrates the theta value of the odometer.
 * 
 * @author Alexander Bratyshkin
 */
public class USLocalizer {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};

	// CONSTANTS
	private static final float ROTATION_SPEED = 200;
	private static final double DISTANCE_FROM_WALL = 6;
	private static final double CLIPPING_THRESHOLD = 0.5;
	private static final float CLIPPING_DISTANCE = 0.60f;
	private static final int BACK_ANGLE = 225;
	private static final int FRONT_ANGLE = 45;
	private static final int ZERO = 0;
	private static final double ZERO_X = 0.0;
	private static final double ZERO_Y = 0.0;
	private static final int HALF = 2;
	
	private Odometer odo;
	private static SampleProvider usSensor;
	private static float[] usData;
	private LocalizationType locType;
	private USPoller us; 

	
	/**
	 * @param odo system odometer
	 * @param us Lower ultrasonic sensor
	 * @param locType type of localization to be performed
	 */
	public USLocalizer(Odometer odo, USPoller us, LocalizationType locType) {
		this.odo = odo;
		this.us = us;
		this.locType = locType;
	}

	
	
	/**
	 * This method localizes the robot with respect to the right angle of a corner of the board. In order to do this,
	 * the robot rotates on its axis constantly recording the distance read by the 
	 * 
	 * @param navigator System navigator
	 */
	public void doLocalization(Navigation navigator) {
		double angleA, angleB, deltaTheta;

		if (locType == LocalizationType.FALLING_EDGE) {

			// ROTATE UNTIL WE ARE FACING NO WALL
			while (us.getDistance() < 40) {
				navigator.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
			}

			// ROTATE UNTIL FIRST WALL IS SEEN BY SENSOR
			while (us.getDistance() > 15) {
				navigator.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
			}

			// STOP MOTORS AND CALCULATE ANGLE OF RIGHT WALL
			navigator.setSpeeds(ZERO, ZERO);
			angleA = odo.getTheta();

			// GO AWAY FROM WALL
			while (us.getDistance() < 15) {
				navigator.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}

			// ROTATE UNTIL SECOND WALL IS SEEN
			while (us.getDistance() > 15) {
				navigator.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}

			// STOP AND RECORD ANGLE OF LEFT WALL
			navigator.setSpeeds(ZERO, ZERO);
			angleB = odo.getTheta();

			// ADJUST THE DELTA THETA BASED ON THE RELATIVE POSITION BETWEEN
			// ANGLE A AND B
			deltaTheta = ((angleA < angleB) ? (BACK_ANGLE - (angleA + angleB) / HALF)
					: (FRONT_ANGLE - (angleA + angleB) / HALF));

			double correctTheta = odo.getTheta() + deltaTheta;

			// SET NEW POSITION ON ODOMETER
			odo.setPosition(new double[] { ZERO_X, ZERO_Y, correctTheta }, new boolean[] { false, false, true });

		} else {

			// ROTATE UNTIL WE SEE A WALL
			while (getFilteredData() > DISTANCE_FROM_WALL) {
				navigator.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
			}

			sleepThread();

			// KEEP ROTATING UNTIL NO WALL IS SEEN
			while (getFilteredData() < DISTANCE_FROM_WALL) {
				navigator.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
			}

			// STOP MOTORS AND RECORD FIRST ANGLE
			navigator.setSpeeds(ZERO, ZERO);
			angleA = odo.getTheta();

			sleepThread();

			// WAIT UNTIL NO WALL IS SEEN
			while (getFilteredData() > DISTANCE_FROM_WALL) {
				navigator.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}


			// ROTATE UNTIL THE ROBOT SEES NO WALL
			while (getFilteredData() < DISTANCE_FROM_WALL) {
				navigator.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}

			// STOP MOTOR AND RECORD SECOND ANGLE
			navigator.setSpeeds(ZERO, ZERO);
			angleB = odo.getTheta();

//			sleepThread();

			// ADJUST THE DELTA THETA BASED ON THE RELATIVE POSITION BETWEEN
			// ANGLE A AND B (CLOCKWISE)
			deltaTheta = ((angleA < angleB) ? (FRONT_ANGLE - (angleA + angleB) / HALF)
					: (BACK_ANGLE - (angleA + angleB) / HALF));

			double correctedTheta = odo.getTheta() + deltaTheta;

			// SET NEW POSITION ON ODOMETER
			this.odo.setPosition(new double[] { ZERO_X, ZERO_Y, correctedTheta }, new boolean[] { false, false, true });

		}

		// TURN TO THE NEW ORIGIN
		navigator.setSpeeds(ROTATION_SPEED, ROTATION_SPEED);
		navigator.turnTo(ZERO, true);
	}

	/**
	 * Clipper for getting the data of the ultrasonic sensor
	 * 
	 * @return filtered distance 
	 */
	public static float getFilteredData() {
		usSensor.fetchSample(usData, ZERO);
		float distance = usData[0];
		// filter
		if (distance > CLIPPING_THRESHOLD) {
			distance = CLIPPING_DISTANCE;
		}

		return distance;
	}

	/**
	 * Sleeps the thread
	 */
	public static void sleepThread() {
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
	}

}
