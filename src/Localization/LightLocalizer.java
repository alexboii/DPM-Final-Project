package Localization;
import Navigation.Navigation;
import Odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	private boolean lineDetected = false;

	// CONSTANTS
	private double SENSOR_TO_AXLE = 7.3;
	private static final float ROTATION_SPEED = 30;
	private static final float SECOND_ROTATION_SPEED = 30;
	private static final int INITIAL_ANGLE = 45;
	private static final int ZERO = 0;
	private static final int MAX_LINE_COUNT = 4;
	private static final int HALF = 2;
	private static final int SCALE_FACTOR = 100;
	private static final int LIGHT_THRESHOLD = 25;
	private static final int ANGLE_CONSTANT = 270;
	private static final int ZERO_X = 0;
	private static final int ZERO_Y = 0;
	private static final double DISTANCE_CONSTANT = 1.4;

	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}

	public void doLocalization(Navigation navigator) {

		// TURN TO 45 DEGREES AS REQUESTED IN THE INSTRUCTIONS
		navigator.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
		navigator.turnTo(INITIAL_ANGLE, true);
		USLocalizer.sleepThread();

		// GO FORWARD ON A 45 DEGREE ANGLE, AND DO SO UNTIL THE SENSOR DETECS A
		// LINE
		navigator.setSpeeds(SECOND_ROTATION_SPEED, SECOND_ROTATION_SPEED);

		while (!lineCrossed())
			;

		// STOP ONCE LINE IS SPOTTED
		navigator.setSpeeds(ZERO, ZERO);

		// ADJUST CENTER OF ROTATION TO DESIRED (0, 0) VALUE
		navigator.goForward(DISTANCE_CONSTANT*SENSOR_TO_AXLE);

		// COUNTS THE LINES WHICH HAVE BEEN CROSSED
		int lineCounter = 0;
		// ALLOWS US TO GET THE POSITION FROM THE ODOMETER
		double[] getPosition = new double[3];
		// RECORDS THE ANGLE EACH TIME A LINE IS DETECTED
		double[] theta = new double[4];

		// ROTATE LEFT (COULD HAVE ROTATED RIGHT)
		navigator.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);

		while (lineCounter < MAX_LINE_COUNT) {
			odo.getPosition(getPosition);
			if (lineCrossed()) {
				// RECORD THE ANGLE EACH TIME A BLACK LINE IS SPOTTED
				theta[lineCounter] = odo.getTheta();
				lineCounter++;
				Sound.beep();
				USLocalizer.sleepThread();
			}
		}

		// STOP THE ROBOT
		navigator.setSpeeds(ZERO, ZERO);

		// FIND THE DIFFERENCE BETWEEN THE ANGLES ON THE Y AND X AXIS
		// RESPECTIVELY
		double lastTheta = theta[3];
		double thetaY = theta[3] - theta[1];
		double thetaX = theta[2] - theta[0];

		// DO COMPUTATIONS PROVIDED IN
		double deltaX = -SENSOR_TO_AXLE * Math.cos(Math.toRadians(thetaY) / HALF);
		double deltaY = -SENSOR_TO_AXLE * Math.cos(Math.toRadians(thetaX) / HALF);
		double deltaTheta = ANGLE_CONSTANT - lastTheta + thetaY / HALF;

		// WRAP ANGLE TO FIT POSITIVE Y AXIS
//		if (deltaTheta > 180) {
//			deltaTheta += 180;
//		}


		// SET NEW POSITION ON ODOMETER
		this.odo.setPosition(new double[] { deltaX, deltaY, odo.getTheta() + deltaTheta },
				new boolean[] { true, true, true });

		// TRAVEL TO DESIRED (0, 0) POINT
		navigator.travelTo(ZERO_X, ZERO_Y);
		USLocalizer.sleepThread();

		// ADJUST TO CORRECT Y+ AXIS
		navigator.turnTo(ZERO, true);

	}

	// GET VALUE FROM SENSOR, TAKEN FROM LAB2
	float setupLightSensor() {
		colorData = new float[colorSensor.sampleSize()];
		colorSensor.fetchSample(colorData, ZERO);
		float lightIntensity = colorData[0] * SCALE_FACTOR;

		return lightIntensity;
	}

	// RETURN TRUE IF THE SENSOR HAS DETECTED A LINE
	private boolean lineCrossed() {
		double lightValue = setupLightSensor();
		boolean newLineDetected = lightValue <= LIGHT_THRESHOLD;
		boolean crossed = !lineDetected && newLineDetected;
		lineDetected = newLineDetected;
		return crossed;
	}

}
