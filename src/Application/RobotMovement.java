package Application;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import Navigation.Navigation;
import Odometer.Odometer;
import SensorData.LSPoller;
import SensorData.USPoller;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

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

	Odometer odometer;
	Navigation navigator;
	USPoller usPollerLow;
	USPoller usPollerHigh;
	protected boolean blue_found = false;
	public static final int ROTATE_SPEED = 100;
	private static final int BAND_WIDTH = 15;

	private static final int TURN_ANGLE_1 = 75;
	private static final int TURN_ANGLE_2 = 100;
	private static final int TURN_ANGLE_3 = -40;
	private static final int RIGHT_ANGLE = 90;
	private static final int SENSOR_ROTATE = 80;
	private static final int FORWARD_DISTANCE = 10;
	private static final int OBJECT_DISTANCE = 25;
	private static final int DISTANCE_COUNTER_LIMIT = 10;
	private static final int FORWARD_SPEED = 200;
	private static final int BB_FAST_SPEED = 260;
	private static final int BB_SLOW_SPEED = 115;
	private static final int ANGLE_LIMIT_BB = 90;
	private static final int ANGLE_LIMIT = 200;
	private static final int DISTANCE_OFFSET = 5;
	private static final int BB_OFFSET = 3;
	private static final int WAYPOINT_X = -60;
	private static final int WAYPOINT_Y = 70;
	private static final int CAGE_FULL_DOWN = 1200;

	EV3LargeRegulatedMotor clawMotor;
	EV3LargeRegulatedMotor pulleyMotor;

	private final double DISTANCE_THRESHOLD_LOW = 20;
	private final double DISTANCE_THRESHOLD_UP = 6;

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
			EV3LargeRegulatedMotor clawMotor, EV3LargeRegulatedMotor pulleyMotor) {
		this.navigator = navigator;
		this.odometer = odometer;
		this.usPollerLow = usPollerLow;
		this.usPollerHigh = usPollerHigh;
		this.clawMotor = clawMotor;
		this.pulleyMotor = pulleyMotor;

	}

	/**
	 * {@inheritDoc}
	 */
	public void run() {

		pulleyMotor.rotate(CAGE_FULL_DOWN);
		clawMotor.rotate(-50);

		// clawMotor.rotate(50);
		// pulleyMotor.rotate(100);
		// clawMotor.rotate(-50);
		// pulleyMotor.rotate(-100);

		while (!blue_found) {

			navigator.turnTo(TURN_ANGLE_1, true);

			navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

			ArrayList<Vector> list_of_vectors = new ArrayList<Vector>();

			while (odometer.getTheta() < ANGLE_LIMIT) {
				navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

				Vector vector = new Vector(usPollerLow.getDistance(), odometer.getTheta());
				list_of_vectors.add(vector);

			}

			Collections.sort(list_of_vectors, new Comparator<Vector>() {
				@Override
				public int compare(Vector a, Vector b) {
					return Double.compare(a.getDistance(), b.getDistance());
				}
			});

			navigator.turnTo(list_of_vectors.get(0).getAngle(), true);
			navigator.goForward((list_of_vectors.get(0).getDistance()));

			if (!isWooden()) {
				clawMotor.rotate(50);
				pulleyMotor.rotate(100);
				clawMotor.rotate(-50);
				pulleyMotor.rotate(-100);
				blue_found = true;
			}

			navigator.goForward(10);

		}

		navigator.turnTo(45, true);
		navigator.travelTo(100, 100);
	}

	private boolean isWooden() {
		return usPollerLow.getDistance() < DISTANCE_THRESHOLD_LOW && usPollerHigh.getDistance() < DISTANCE_THRESHOLD_UP;
	}

}
