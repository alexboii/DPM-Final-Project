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
 * This class essentially coordinates all of the robot's operations. It
 * executes our search algorithm to find an object in the vecinity. If the object is
 * a wooden block, it avoids it, goes to the next waypoint and keeps looking for objects.
 * 
 * If the found object is a blue block, it activates the claw mechanism to catch it. If it is not
 * the first blue block the robot encounters, previously picked up blocks are placed on top of the 
 * new block. If the maximum amount of blue blocks is  not reached, it goes to the next way point
 * and scans again. Otherwise, robot goes to drop off zone, drops the blocks, and goes back to its 
 *  position.
 * 
 * 
 * @author Alexander Bratyshkin
 * @author Sebastian Andrade
 *
 */
/**
 * @author root
 *
 */
/**
 * @author root
 *
 */
public class RobotMovement extends Thread {

	static Odometer odometer;
	static Navigation navigator;
	USPoller usPollerLow;
	USPoller usPollerHigh;

	ArrayList<Vector> visited_waypoints = new ArrayList<Vector>();
	public static int wayPointCounter = 0;

	public static int STACK_HEIGHT = 2;

	protected boolean blue_found = false;
	public static final int ROTATE_SPEED = 76;

	private static final double TILE = 30.48;

	private static final int TURN_ANGLE_1 = 45;
	private static final int ANGLE_LIMIT = 180;

	private static final int PULLEY_SPEED = -500;
	private static final int PULL_DOWN_FULL = 1200;
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

		clawMotor.setAcceleration(200);

	}

	/**
	 * {@inheritDoc}
	 */ public void run() { 
		clawMotor.rotate(OPEN_CLAW_1);

		//Move away from wall before first scan
		navigator.turnTo(180, true);
		navigator.goForward(TILE);
		navigator.turnTo(90, true);

		
		//look for objects until max capacity is reached
		while (blue_counter < STACK_HEIGHT) {
			
			//Look for objects in the range of (TURN_ANGLE_1 to ANGLE_LIMIT) 
			findObject(TURN_ANGLE_1, ANGLE_LIMIT);
		}

		//Goes to drop off zone & drops the blocks
		goToDropOffZone();
		pullCageDown();
		clawMotor.rotate(OPEN_CLAW_1);

	}

	/**
	 * Calculates the middle of the the designated drop off zone & goes to that point.
	 */
	public void goToDropOffZone() {
		double x, y;

		x = -TILE * average(StartRobot.UDZx, StartRobot.LDZx);
		y = TILE * average(StartRobot.UDZy, StartRobot.LDZy);

		navigator.travelTo(x, y, false);

	}

	/**
	 * @param double a
	 * @param double b
	 * @return average of both input arguments
	 */
	public static double average(double a, double b) {
		return Math.abs(((Math.abs(a) + Math.abs(b)) / 2));
	}

	/**
	 * Calculates X coordinates of a point based on distance from robot and angle
	 * @param distance to desired point
	 * @param angle: heading where the point is located relative to the robot
	 * @return absolute X value of desired point
	 */
	public static double getXWP(double distance, double angle) {
		double x = Math.abs(odometer.getX()) + distance * Math.cos(Math.toRadians(180 - angle));
		return x;
	}

	/**
	 * Calculates Y coordinates of a point based on distance from robot and angle
	 * @param distance to desired point
	 * @param angle: heading where the point is located relative to the robot
	 * @return absolute Y value of desired point
	 */
	public static double getYWP(double distance, double angle) {
		double y = Math.abs(odometer.getY()) + distance * Math.cos(Math.toRadians(angle - 90));
		return y;
	}
	
	
	

	/**
	 * Robot goes to next way point. The next way point is located along the line between the robot's current
	 * position and the drop off zone.
	 */
	public static void goToNextWayPoint() {

		double data[] = new double[2];
		double x, y;
		double finalX, finalY;
		double distance;
		double angle;
		
		// calculate middle of green zone
		finalX = TILE * average(StartRobot.UDZx, StartRobot.LDZx);
		finalY = TILE * average(StartRobot.UDZy, StartRobot.LDZy);

		//Get Angle of next waypoint
		angle = navigator.getMinAng(-finalX, finalY);

		//Distance from current position to next waypoint
		distance = TILE + 2 - DISTANCE_APPROACH_THRESHOLD;

		
		//Get XY points of next way point
		x = getXWP(distance, angle);
		y = getYWP(distance, angle);

		//Tries to go to the next way point
		//If it encounters an object on the way  & it's a wooden block, 
		// it avoids it and goes to a new way point
		//If object is a blue block, it picks it up &
		//the new way point is the current position
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
	}

	
	/**
	 * The robot rotates in its axis starting at  theta=initialAngle and finishing at theta = finalAngle
	 * While scaning, every ADDITION_SLEEP_TIME miliseconds, a Vector object is created with the lower US sensor reading
	 * as length of the vector and the odometer's theta as the angle. When scan is done, the Vector with the 
	 * smallest length is selected. Robot turns to that Vector's angle and moves forward by a distance equal to 
	 * (Vector's length - SCAN_DISTANCE_THRESHOLD). 
	 * Once in front of the object, the robot scans it. If it's a blue block, it picks it up and either keeps 
	 * scanning or goes to the drop off zone. If object is a wooden block, the robot avoids it and goes to
	 * a new way point. 
	 * @param initialAngle Starting angle for scan
	 * @param finalAngle	Ending angle for scan
	 */
	public void findObject(double initialAngle, double finalAngle) {
		ArrayList<Vector> list_of_vectors = new ArrayList<Vector>();

		double[] blockProperties = new double[2];

		//Turn to initial angle to start scan
		navigator.turnTo(initialAngle, true);
		
		//Rotate on its axis
		navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

		
		//Keep rotating till reach finalAngle
		while (odometer.getTheta() < finalAngle) {
			navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

			//Create all the Vector objects to be analyzed
			Vector vector = new Vector(usPollerLow.getDistance(), odometer.getTheta(), odometer.getX(),
					odometer.getY());

			//Add Vector object to the list
			list_of_vectors.add(vector);

			//Wait to avoid creating too many Vector Objects
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

		

		// if closest object is not too far away, analyze it
		if ((list_of_vectors.get(0).getDistance()) < DISTANCE_SCAN_THRESHOLD) {

			//Turn to the object's heading
			navigator.turnTo(list_of_vectors.get(0).getAngle(), true);

			
			//If lower ultrasonic sensor detects the object, approach it based on that reading
			if (usPollerLow.getDistance() < DISTANCE_SCAN_THRESHOLD) {
				navigator.goForward((usPollerLow.getDistance()) - DISTANCE_APPROACH_THRESHOLD);
			} else {
				//Otherwise, approach object based on Vector object
				navigator.goForward(list_of_vectors.get(0).getDistance() - DISTANCE_APPROACH_THRESHOLD);
			}

			list_of_vectors.clear();

			//Identify object as wooden block or blue block
			blockProperties = navigator.isWooden();


			// IF its a wooden block, avoid it
			if (blockProperties[0] == 1) {
				navigator.avoidObject(false);
				goToNextWayPoint();
				return;

			} else { //Its blue block

				//If supposed blue block is close & still looking for blue objects
				if (blockProperties[1] < DISTANCE_APPROACH_THRESHOLD + 10) {
					if (blue_counter < STACK_HEIGHT) {

						//Aproach object
						navigator.goForward(DISTANCE_APPROACH_THRESHOLD + 1);
						
						//Pick up block
						pickUpBlock();

						return;

					}
				} else { // if detected as block but nothing in front
					navigator.goForward(0.3 * DISTANCE_APPROACH_THRESHOLD);
				}
			}
		} else { // objects not close enough but in sight, the robot approaches it & scans again
			if (list_of_vectors.get(0).getDistance() < 1.5 * TILE) {
				navigator.turnTo(list_of_vectors.get(0).getAngle(), true);
				navigator.goForward(TILE * 0.8);
				return;
			}
		}

		//If still looking for blocks, go to next way point
		if (blue_counter < STACK_HEIGHT) {
			goToNextWayPoint();
		}
	}

	/**
	 * This method picks up a block. If it's not the first block, previously picked up blocks
	 * are placed on top of the new block.	
	 */
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

	/**
	 * Pulls down the claw to pick up blocks
	 */
	private static void pullCageDown() {
		pulleyMotor.setSpeed(PULLEY_SPEED);
		pulleyMotor.rotate((int) (PULL_DOWN_FULL));
	}
	
	/**
	 * It lowers the open claw to the the lowest point, closes it & raises the
	 * claw to its maximum height.
	 */
	private static void grabObject() {
		clawMotor.setAcceleration(200);
		pulleyMotor.rotate(PULL_DOWN_TO_BLOCK);
		clawMotor.rotate(CLOSE_CLAW_2);
		pulleyMotor.rotate(PULL_UP_FROM_BLOCK);
	}

}
