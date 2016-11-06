package Localization;
import java.util.Arrays;
import java.util.LinkedList;

import Navigation.Navigation;
import Odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;

public class USLocalizer {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};

	// CONSTANTS
	private static final float ROTATION_SPEED = 100;
	private static final double DISTANCE_FROM_WALL = 0.5;
	private static final double CLIPPING_THRESHOLD = 0.5;
	private static final float CLIPPING_DISTANCE = 0.60f;
	private static final int BACK_ANGLE = 225;
	private static final int FRONT_ANGLE = 45;
	private static final int ZERO = 0;
	private static final double ZERO_X = 0.0;
	private static final double ZERO_Y = 0.0;
	private static final int HALF = 2;
	private static final int SLEEP_TIME = 1000;
	
	private Odometer odo;
	private static SampleProvider usSensor;
	private static float[] usData;
	private LocalizationType locType;

	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData, LocalizationType locType) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
	}

	public void doLocalization(Navigation navigator) {
		double[] pos = new double[3];
		double angleA, angleB, deltaTheta;

		if (locType == LocalizationType.FALLING_EDGE) {

			// ROTATE UNTIL WE ARE FACING NO WALL
			while (getFilteredData() < DISTANCE_FROM_WALL) {
				navigator.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
			}

			// WE SLEEP THE THREAD BETWEEN EACH STEP TO IMPROVE ACCURACY AND
			// REDUCE SLIPPING
			sleepThread();

			// ROTATE UNTIL FIRST WALL IS SEEN BY SENSOR
			while (getFilteredData() > DISTANCE_FROM_WALL) {
				navigator.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
			}

			// STOP MOTORS AND CALCULATE ANGLE OF RIGHT WALL
			navigator.setSpeeds(ZERO, ZERO);
			angleA = odo.getTheta();

			sleepThread();

			// GO AWAY FROM WALL
			while (getFilteredData() < DISTANCE_FROM_WALL) {
				navigator.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}

			sleepThread();

			// ROTATE UNTIL SECOND WALL IS SEEN
			while (getFilteredData() > DISTANCE_FROM_WALL) {
				navigator.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}

			// STOP AND RECORD ANGLE OF LEFT WALL
			navigator.setSpeeds(ZERO, ZERO);
			angleB = odo.getTheta();

			sleepThread();

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

			sleepThread();

			// ROTATE UNTIL THE ROBOT SEES NO WALL
			while (getFilteredData() < DISTANCE_FROM_WALL) {
				navigator.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
			}

			// STOP MOTOR AND RECORD SECOND ANGLE
			navigator.setSpeeds(ZERO, ZERO);
			angleB = odo.getTheta();

			sleepThread();

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

	// CLIPPER FOR GETTING THE DATA OF THE US SENSOR
	public static float getFilteredData() {
		usSensor.fetchSample(usData, ZERO);
		float distance = usData[0];
		// filter
		if (distance > CLIPPING_THRESHOLD) {
			distance = CLIPPING_DISTANCE;
		}

		return distance;
	}

	// SLEEP THE THREAD FOR 1 SECOND
	public static void sleepThread() {
		try {
			Thread.sleep(SLEEP_TIME);
		} catch (InterruptedException e) {
		}
	}

}
