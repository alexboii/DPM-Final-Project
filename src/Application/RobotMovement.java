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
	
	ArrayList<Vector> visited_waypoints = new ArrayList<Vector>();
	public static Vector lastWayPoint;
	

	
	protected boolean blue_found = false;
	public static final int ROTATE_SPEED = 76;
	private static final int BAND_WIDTH = 15;
	
	private static final double TILE = 30.48;

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

	private static final int PULLEY_SPEED = -500;
	private static final int PULL_DOWN_FULL = 1200;
	private static final int CLOSE_CLAW_1 = -40;
	private static final int OPEN_CLAW_1 = 100;
	private static final int OPEN_CLAW_2 = 20;
	private static final int OPEN_CLAW_3 = 80;
	private static final int PULL_DOWN_TO_BLOCK = 700;
	private static final int CLOSE_CLAW_2 = -200;
	private static final int PULL_UP_FROM_BLOCK = -2000;
	private static final int DISTANCE_SCAN_THRESHOLD = 41;
	private static final int DISTANCE_APPROACH_THRESHOLD = 8;
	private static final int ADDITION_SLEEP_TIME = 30;


	EV3LargeRegulatedMotor clawMotor;
	EV3LargeRegulatedMotor pulleyMotor;

	private final double DISTANCE_THRESHOLD_LOW = 20;
	private final double DISTANCE_THRESHOLD_UP = 6;
	private int blue_counter = 0;

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
	clawMotor.rotate(OPEN_CLAW_1);

		while (blue_counter < 4) {
			lastWayPoint = new Vector(usPollerLow.getDistance(), odometer.getTheta(), odometer.getX(), odometer.getY());

			findObject(TURN_ANGLE_1);
			
			goToNextWayPoint();
			
		}
		
	//	navigator.travelTo(-100, 100, true);
		navigator.travelTo((StartRobot.UGZx + StartRobot.LGZx)/2 , (StartRobot.UGZy + StartRobot.LGZy)/2 , true);
		pullCageDown();
		clawMotor.rotate(OPEN_CLAW_1);
//		clawMotor.rotate(OPEN_CLAW_1);

	}

	public void goToNextWayPoint(){
		
		double x, y;
		
		if((int)(odometer.getX()/TILE) - 1 < Navigation.MAX_X_BOARD ){
			x = odometer.getX() - TILE;
			y = odometer.getY();
		} else { 
			x = odometer.getX();
			y = odometer.getY() + TILE;
		}
		
		navigator.travelTo(x,y, true);	
	}
	
	
	public void findObject(double angle){
		ArrayList<Vector> list_of_vectors = new ArrayList<Vector>();
		
		double[] blockProperties = new double[2];


		navigator.turnTo(angle, true);
		navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);


		while (odometer.getTheta() < ANGLE_LIMIT) {
			navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

			Vector vector = new Vector(usPollerLow.getDistance(), odometer.getTheta(), odometer.getX(), odometer.getY());

			list_of_vectors.add(vector);

			try {
				Thread.sleep(ADDITION_SLEEP_TIME);
			} catch (InterruptedException e) {
			}
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
		
		if ((list_of_vectors.get(0).getDistance()) < DISTANCE_SCAN_THRESHOLD) {

			navigator.turnTo(list_of_vectors.get(0).getAngle(), true);
			navigator.goForward((list_of_vectors.get(0).getDistance()) - DISTANCE_APPROACH_THRESHOLD);
			
			blockProperties = navigator.isWooden();
					
					
					if (blockProperties[0] == 1) { //if wooden block
						navigator.avoidObject();
						return;
						
					} else {
						if(blockProperties[1] < 2 * DISTANCE_APPROACH_THRESHOLD){
							navigator.goForward(DISTANCE_APPROACH_THRESHOLD-3);
							pullCageDown();
		
							if(!(blue_counter == 0)){
								clawMotor.rotate(OPEN_CLAW_2);
								clawMotor.rotate(OPEN_CLAW_3);
							}
							grabObject();
							list_of_vectors.clear();
							blue_counter++;
							} else { return; } 
					}
		}
		Sound.beepSequenceUp();
		list_of_vectors.clear();
	
	}
	
	

	private void pullCageDown() {
		pulleyMotor.setSpeed(PULLEY_SPEED);
		pulleyMotor.rotate(PULL_DOWN_FULL);
	}

	private void grabObject() {
//		clawMotor.rotate(OPEN_CLAW_1);
		clawMotor.setAcceleration(5000);
		pulleyMotor.rotate(PULL_DOWN_TO_BLOCK);
		clawMotor.rotate(CLOSE_CLAW_2);
		pulleyMotor.rotate(PULL_UP_FROM_BLOCK);
	}

}
