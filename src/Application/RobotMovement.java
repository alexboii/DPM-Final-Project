package Application;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import Navigation.Navigation;
import Odometer.Odometer;
import Odometer.LCDInfo;
import SensorData.USPoller;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class essentially coordinates all of the robot�s operations. It
 * executes our search algorithm to find an object in its path, from which we
 * then snap into the Navigation class once the maximum amount of blue blocks
 * has been picked up to travel to the designated zone and then restart the
 * algorithm again. The differentiation between the type of blocks will also be
 * done in this class, as well as the control of the pulley motor and the claw.
 * 
 * @author Alex
 *
 */
public class RobotMovement extends Thread {

	static Odometer odometer;
	static Navigation navigator;
	USPoller usPollerLow;
	USPoller usPollerHigh;

	private SampleProvider colorSensor;
	private float[] colorData;

	private static int numberOfScans = 0;

	private static double lastMinLow = 255;

	ArrayList<Vector> visited_waypoints = new ArrayList<Vector>();
	// public static Vector lastWayPoint;
	public static int wayPointCounter = 0;

	public static int STACK_HEIGHT = 2;

	private static final double LIGHT_SENSOR_OFFSET = 11;
	private static final int LIGHT_THRESHOLD = 35;

	protected boolean blue_found = false;
	public static final int ROTATE_SPEED = 76;

	private static final double TILE = 30.48;

	private static final int TURN_ANGLE_1 = 45;
	private static final int ANGLE_LIMIT = 180;

	private static final int PULLEY_SPEED = -500;
	private static final int PULL_DOWN_FULL = 1200;
	private static final int CLOSE_CLAW_1 = -40;
	private static final int OPEN_CLAW_1 = 100;
	private static final int OPEN_CLAW_2 = 30;
	private static final int OPEN_CLAW_3 = 80;
	private static final int PULL_DOWN_TO_BLOCK = 690;
	private static final int CLOSE_CLAW_2 = -(OPEN_CLAW_1 + OPEN_CLAW_2 + OPEN_CLAW_3);
	private static final int PULL_UP_FROM_BLOCK = -1900;
	private static final int DISTANCE_SCAN_THRESHOLD = (int) TILE;
	public static final int SECOND_DISTANCE_SCAN = (int) (1.5 * TILE);

	private static final int DISTANCE_APPROACH_THRESHOLD = 5;
	private static final int ADDITION_SLEEP_TIME = 15;

	static EV3LargeRegulatedMotor clawMotor;
	static EV3LargeRegulatedMotor pulleyMotor;

	private static int blue_counter = 0;

	/**
	 * Constructor
	 * 
	 * @param odometer
	 *            Odometer
	 * @param navigator
	 *            Navigator
	 * @param usPoller
	 *            Ultrasonic Sensor Poller
	 * @param lsPoller
	 *            Light Sensor Poller
	 * @param usMotor
	 *            Ultrasonic Sensor Motor
	 */
	public RobotMovement(Odometer odometer, Navigation navigator, USPoller usPollerLow, USPoller usPollerHigh,
			EV3LargeRegulatedMotor clawMotor, EV3LargeRegulatedMotor pulleyMotor, SampleProvider colorSensor,
			float[] colorData) {
		RobotMovement.navigator = navigator;
		RobotMovement.odometer = odometer;
		this.usPollerLow = usPollerLow;
		this.usPollerHigh = usPollerHigh;
		RobotMovement.clawMotor = clawMotor;
		RobotMovement.pulleyMotor = pulleyMotor;
		this.colorSensor = colorSensor;
		this.colorData = colorData;

		clawMotor.setAcceleration(200);

	}

	/**
	 * {@inheritDoc}
	 */
	public void run() {
		// boolean isWooden;

		// pullCageDown();
		clawMotor.rotate(OPEN_CLAW_1);

		navigator.turnTo(180, true);
		navigator.goForward(TILE);
		navigator.turnTo(90, true);

		while (blue_counter < STACK_HEIGHT) {
			// Sound.beep();
			// lastWayPoint = new Vector(usPollerLow.getDistance(),
			// odometer.getTheta(), odometer.getX(), odometer.getY());

			if (odometer.needsCalibration()) {
				calibrateOdometer();
				Sound.beepSequenceUp();
				odometer.setCalibration(false);
			}

			findObject(TURN_ANGLE_1, ANGLE_LIMIT);

			// if(blue_counter < STACK_HEIGHT){
			// goToNextWayPoint();
			// }
			// navigator.travelTo(lastWayPoint.getInitialX() - TILE,
			// lastWayPoint.getInitialY(), true);
			// Sound.beep();

		}

		// Sound.beepSequence();
		goToDropOffZone();
		pullCageDown();
		clawMotor.rotate(OPEN_CLAW_1);

	}

	public void goToDropOffZone() {
		double x, y;

		x = -TILE * average(StartRobot.UDZx, StartRobot.LDZx);
		y = TILE * average(StartRobot.UDZy, StartRobot.LDZy);

		navigator.travelTo(x, y, false);

	}

	public static double average(double a, double b) {
		return Math.abs(((Math.abs(a) + Math.abs(b)) / 2));
	}

	public static double getXWP(double distance, double angle) {
		double x = Math.abs(odometer.getX()) + distance * Math.cos(Math.toRadians(180 - angle));
		// LCDInfo.setLabel1("XW");
		// LCDInfo.setValue1((int)(x));
		return x;
	}

	public static double getYWP(double distance, double angle) {
		double y = Math.abs(odometer.getY()) + distance * Math.cos(Math.toRadians(angle - 90));
		// LCDInfo.setLabel2("YW");
		// LCDInfo.setValue2((int)(y));
		return y;
	}

	public static void goToNextWayPoint() {

		double data[] = new double[2];
		double x, y;
		double finalX, finalY;
		double distance;
		double angle;
		double length;
		// calculate middle of green zone
		finalX = TILE * average(StartRobot.UDZx, StartRobot.LDZx);
		finalY = TILE * average(StartRobot.UDZy, StartRobot.LDZy);

		// calculate distance from current position to middle of green zone
		length = Math.hypot(finalX - Math.abs(odometer.getX()), finalY - odometer.getY());

		angle = navigator.getMinAng(-finalX, finalY);

		distance = TILE + 2 - DISTANCE_APPROACH_THRESHOLD;

		x = getXWP(distance, angle);
		y = getYWP(distance, angle);

		if (!navigator.travelTo(-x, y)) {

			data = navigator.isWooden();
			if (data[0] == 1) { // if wooden
				navigator.avoidObject(true);
				return;
			} else {
				if (blue_counter < STACK_HEIGHT) {
					pickUpBlock();
				}
			}

			angle = navigator.getMinAng(-finalX, finalY);
			x = getXWP(distance, angle);
			y = getYWP(distance, angle);
			navigator.travelTo(-x, y);
		}

		/*
		 * if( wayPointCounter < -Navigation.MAX_X_BOARD - 1 ){
		 * //Sound.beepSequence(); x = lastWayPoint.getInitialX() - TILE; y =
		 * lastWayPoint.getInitialY(); } else { //Sound.beepSequenceUp(); x =
		 * lastWayPoint.getInitialX(); y = lastWayPoint.getInitialY() + TILE; }
		 * // Sound.beep(); ++wayPointCounter; navigator.travelTo(x,y, true); //
		 * Sound.beep();
		 */
	}

	public void findObject(double initialAngle, double finalAngle) {
		++numberOfScans;
		ArrayList<Vector> list_of_vectors = new ArrayList<Vector>();

		double[] blockProperties = new double[2];

		navigator.turnTo(initialAngle, true);
		navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

		while (odometer.getTheta() < finalAngle) {
			navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

			Vector vector = new Vector(usPollerLow.getDistance(), odometer.getTheta(), odometer.getX(),
					odometer.getY());

			list_of_vectors.add(vector);

			try {
				Thread.sleep(ADDITION_SLEEP_TIME);
			} catch (InterruptedException e) {
			}
		} // end scan

		// look for smallest
		Collections.sort(list_of_vectors, new Comparator<Vector>() {
			@Override
			public int compare(Vector a, Vector b) {
				return Double.compare(a.getDistance(), b.getDistance());
			}
		});

		// waits for some reason
		/*
		 * try { Thread.sleep(1000); } catch (InterruptedException e) { }
		 */

		// if closest object is not too far away, analyze it
		if ((list_of_vectors.get(0).getDistance()) < DISTANCE_SCAN_THRESHOLD) {

			navigator.turnTo(list_of_vectors.get(0).getAngle(), true);

			if (usPollerLow.getDistance() < DISTANCE_SCAN_THRESHOLD) {
				navigator.goForward((usPollerLow.getDistance()) - DISTANCE_APPROACH_THRESHOLD);
			} else {
				navigator.goForward(list_of_vectors.get(0).getDistance() - DISTANCE_APPROACH_THRESHOLD);
			}

			list_of_vectors.clear();

			blockProperties = navigator.isWooden();

			// if (blockProperties[0] == 1 ||
			// blockProperties[1] < 2 * DISTANCE_APPROACH_THRESHOLD) { //if
			// wooden block

			// IF its a wooden block
			if (blockProperties[0] == 1) {
				navigator.avoidObject(false);
				// goToNextWayPoint();
				return;

			} else {

				if (blockProperties[1] < DISTANCE_APPROACH_THRESHOLD + 10) {
					if (blue_counter < STACK_HEIGHT) {

						navigator.goForward(DISTANCE_APPROACH_THRESHOLD + 1);
						pickUpBlock();

						return;

					}
				} else { // if detected as block but nothing in front
					// Sound.beepSequenceUp();
					navigator.goForward(0.3 * DISTANCE_APPROACH_THRESHOLD);
				}
			}
		} else { // objects not close enough
			if (list_of_vectors.get(0).getDistance() < 1.5 * TILE) {
				navigator.turnTo(list_of_vectors.get(0).getAngle(), true);
				navigator.goForward(TILE * 0.8);
				return;
			}

		}

		if (blue_counter < STACK_HEIGHT) {
			Sound.beep();
			Sound.beep();
			goToNextWayPoint();
		}

	}

	public static void pickUpBlock() {

		pullCageDown();

		if (!(blue_counter == 0)) {
			clawMotor.setAcceleration(200);
			clawMotor.rotate(OPEN_CLAW_2);
			clawMotor.rotate(OPEN_CLAW_3);
		}
		grabObject();
		blue_counter++;

	}

	private static void pullCageDown() {
		pulleyMotor.setSpeed(PULLEY_SPEED);
		pulleyMotor.rotate((int) (PULL_DOWN_FULL));
	}

	private static void grabObject() {
		// clawMotor.rotate(OPEN_CLAW_1);
		clawMotor.setAcceleration(200);
		pulleyMotor.rotate(PULL_DOWN_TO_BLOCK);
		clawMotor.rotate(CLOSE_CLAW_2);
		pulleyMotor.rotate(PULL_UP_FROM_BLOCK);
	}

	////////// ODOMETRY CORRECTION ATTEMPT

	public void calibrateOdometer() {
		double new_x, new_y;

		navigator.turnTo(90, true);
		navigator.setSpeeds(Navigation.FORWARD_SPEED, Navigation.FORWARD_SPEED);

		while (getLightValue() > LIGHT_THRESHOLD) {
			;
		}
		Sound.beep();

		navigator.setSpeeds(0, 0);

		navigator.turnTo(180, true);

		navigator.setSpeeds(Navigation.FORWARD_SPEED, Navigation.FORWARD_SPEED);

		while (getLightValue() > LIGHT_THRESHOLD) {
			;
		}
		Sound.beep();

		navigator.setSpeeds(0, 0);

		navigator.goForward(LIGHT_SENSOR_OFFSET);

		navigator.turnTo(90, true);

		navigator.goForward(LIGHT_SENSOR_OFFSET);

		new_x = -getLineValue(Math.abs(odometer.getX()));
		new_y = getLineValue(odometer.getY());

		odometer.setPosition(new double[] { new_x, new_y, odometer.getTheta() }, new boolean[] { true, true, false });

	}

	public double getLineValue(double CurrentOdometerValue) {
		return ((int) ((CurrentOdometerValue / TILE) + 0.5)) * TILE;
	}

	// GET VALUE FROM SENSOR, TAKEN FROM LAB2
	float getLightValue() {
		colorData = new float[colorSensor.sampleSize()];
		colorSensor.fetchSample(colorData, 0);
		float lightIntensity = colorData[0] * 100;

		LCDInfo.setLabel3("LS:");
		LCDInfo.setValue3((int) (lightIntensity));

		return lightIntensity;
	}

}
