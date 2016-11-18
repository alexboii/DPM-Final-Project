package Application;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import Navigation.Navigation;
import Odometer.Odometer;
import SensorData.USPoller;
import lejos.hardware.Sound;
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
	public static final int ROTATE_SPEED = 50;
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
	private static final int ANGLE_LIMIT = 180;
	private static final int DISTANCE_OFFSET = 5;
	private static final int BB_OFFSET = 3;
	private static final int WAYPOINT_X = -60;
	private static final int WAYPOINT_Y = 70;
	private static final int CAGE_FULL_DOWN = 1200;

	private static final int pulleySpeed = -500;
	private static final int pullDownFull = 1200;
	private static final int closeClaw_1 = -40;
	private static final int openClaw_1 = 100;
	private static final int pullDownToBlock = 700;
	private static final int closeClaw_2 = -200;
	private static final int pullUpFromBlock = -2000;
	private static final int distanceScanThreshold = 30;
	private static final int distanceApproachThreshold = 11;

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
		boolean isWooden;
	//	pullCageDown();
	clawMotor.rotate(openClaw_1);

		while (!blue_found) {

			navigator.turnTo(TURN_ANGLE_1, true);
			navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

			ArrayList<Vector> list_of_vectors = new ArrayList<Vector>();

			while (odometer.getTheta() < ANGLE_LIMIT) {
				navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

				Vector vector = new Vector(usPollerLow.getDistance(), odometer.getTheta());
				try {
					Thread.sleep(150);
				} catch (InterruptedException e) {
				}
				list_of_vectors.add(vector);

			}

			Collections.sort(list_of_vectors, new Comparator<Vector>() {
				@Override
				public int compare(Vector a, Vector b) {
					return Double.compare(a.getDistance(), b.getDistance());
				}
			});

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
			}
			
			if ((list_of_vectors.get(0).getDistance()) < distanceScanThreshold) {

				navigator.turnTo(list_of_vectors.get(0).getAngle(), true);
				navigator.goForward((list_of_vectors.get(0).getDistance()) - distanceApproachThreshold);
				
				isWooden = navigator.isWooden();
				
				if (isWooden) {
					
					navigator.avoidObject();
					continue;
				} else {
					navigator.goForward(distanceApproachThreshold-3);
					pullCageDown();
					grabObject();
					list_of_vectors.clear();
					blue_found = true;
					break;

				}
				
			}
			Sound.beepSequenceUp();
			list_of_vectors.clear();
			
			navigator.turnTo(135, true);
			navigator.goForward(distanceScanThreshold * 0.5);

		}
		
	//	navigator.travelTo(-100, 100, true);
		navigator.travelTo((StartRobot.UGZx + StartRobot.LGZx)/2 , (StartRobot.UGZy + StartRobot.LGZy)/2 , true);
		clawMotor.rotate(openClaw_1);
//		clawMotor.rotate(openClaw_1);

	}

	private boolean isWooden() {
		return usPollerLow.getDistance() < DISTANCE_THRESHOLD_LOW && usPollerHigh.getDistance() < DISTANCE_THRESHOLD_UP;
	}

	private void pullCageDown() {
		pulleyMotor.setSpeed(pulleySpeed);
		pulleyMotor.rotate(pullDownFull);
//		clawMotor.rotate(closeClaw_1);
	}

	private void grabObject() {
//		clawMotor.rotate(openClaw_1);
		clawMotor.setAcceleration(5000);
		pulleyMotor.rotate(pullDownToBlock);
		clawMotor.rotate(closeClaw_2);
		pulleyMotor.rotate(pullUpFromBlock);
	}

}
