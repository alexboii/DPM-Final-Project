
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigatorObstacle extends Thread {

	// CONSTANTS
	private static final int ROTATE_SPEED = 150;
	private static final int FORWARD_SPEED = 200;
	private static final int ACCELERATION = 4000;
	private static final int WALL_DISTANCE = 8;
	private static final int DISTANCE_ERROR = 1;
	private static final int ZERO = 0;
	private static final double DEGREE_ERROR = Math.toRadians(4.4);
	private static final double RIGHT_ANGLE = Math.toRadians(90);
	private static final double SENSOR_ROTATE_TO_WALL = 70;
	private static final double SENSOR_ROTATE_FROM_WALL = 80;
	private static final int BAND_CENTER = 10;
	private static final double SAFETY_DEGREE_ERROR = Math.toRadians(10);
	private static final int BIG_SLEEP = 2000, SMALL_SLEEP = 20;
	private final int TIMES_TWO = 2;

	// CLASS ATTRIBUTES
	private double changeTheta, dy, dx;
	protected OdometerAvoidance odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor, usMotor;
	private int points[][];
	private double currentX, currentY, currentTheta, distance;
	private double track;
	private USPoller usPoller;
	private boolean isNavigating = false;

	private boolean avoidance;

	// CONSTANTS FOR BANG BANG CONTROLLER
	private final int delta = 175;
	private final int LOW_CONSTANT = 15;
	private static final int motorLow = 100;
	private static final int motorHigh = 260;

	private static final int BAND_WIDTH = 5;

	public NavigatorObstacle(OdometerAvoidance odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			int waypoints[][], EV3LargeRegulatedMotor usMotor, USPoller usPoller, boolean avoidance,
			double track) {

		this.usPoller = usPoller;
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.points = waypoints;
		this.avoidance = avoidance;
		this.track = track;
		this.usMotor = usMotor;

		this.currentX = odometer.getX();
		this.currentY = odometer.getY();
		this.currentTheta = odometer.getTheta();
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);

	}

	public void run() {

		// WAIT A FEW SECONDS BEFORE INITIALIZING THE MOVEMENT
		try {
			Thread.sleep(BIG_SLEEP);
		} catch (InterruptedException e) {

		}

		// TRAVEL EACH OF THE DEFINED POINTS, ONE POINT AT A TIME
//		System.out.println("\n\n\n\n" + points[0][0] + "//" + points[0][1]);
		travelTo(points[0][0], (points[0][1]));
		// STOP THE MOTORS ONCE IT'S DONE
		leftMotor.stop();
		rightMotor.stop();

	}

	public void travelTo(double x, double y) {

		// DO NOT STOP TRAVELLING UNTIL YOU ARE WITHIN 1 CM FROM DESIRED POINT
		while (Math.hypot(currentX - x, currentY - y) > DISTANCE_ERROR) {

			// CALCULATION OF DESTINATION BASED ON OUR CURRENT POSITION
			isNavigating = true;
			dy = y - currentY;
			dx = x - currentX;

			// CALCULATE ROTATION ANGLE BASED ON THE CURRENT POSITION
			// ANGLE ALREADY CAPPED BETWEEN -PI AND PI
			double orientation = Math.atan2(dx, dy);

			// WRAP ANGLE WITHIN 0 AND 2PI
			if (orientation < ZERO) {
				orientation += Math.PI * TIMES_TWO;
			}

			// ERROR OF ANGLE IN REGARDS TO THE EXPECTED ORIENTATION
			double error = Math.abs(orientation - currentTheta);

			// ROTATE ROBOT TO DESIRED ANGLE (WITHIN THRESHOLD)
			if (error > DEGREE_ERROR) {
				turnTo(orientation, currentTheta);
			}

			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);

			// CALCULATE THE DISTANCE THAT THE ROBOT NEEDS TO TRAVEL
			distance = Math.hypot(dx, dy);

			// IF WE HAVE TO DO AVOIDANCE
			if (avoidance) {

				// TRAVEL THE CALCULATED DISTANCE ABOVE
				leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, distance), true);
				rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, distance), true);

				// WHILE THE ROBOT IS MOVING, KEEP CHECKING FOR OBJECTS THAT
				// MIGHT OBSTRUCT MOVEMENT
				moving_loop: while (isMoving()) {

					// IF THE SENSOR DETECTS AN OBJECT
					if (usPoller.getDistance() < WALL_DISTANCE) {
						// STOP MOTORS
						leftMotor.stop();
						rightMotor.stop();

						// MAKE SHARP 90 DEGREES ROTATION
						leftMotor.rotate(-convertAngle(Lab5.WHEEL_RADIUS, track, RIGHT_ANGLE), true); 																				// (1.57)
						rightMotor.rotate(convertAngle(Lab5.WHEEL_RADIUS, track, RIGHT_ANGLE), false); 

						// ROTATE SENSOR TO FACE WALL
						usMotor.rotateTo((int) SENSOR_ROTATE_TO_WALL);

						// ENTER INTO BANG BANG CONTROLLER MODE
						wallfollower_loop: while (true) {

							int bang_bang_error = (int) (usPoller.getDistance() - BAND_CENTER);

							// ROBOT IS AT THE RIGHT DISTANCE FROM WALL, KEEP
							// GOING
							if (Math.abs(bang_bang_error) <= BAND_WIDTH) {
								rightMotor.setSpeed(FORWARD_SPEED);
								leftMotor.setSpeed(FORWARD_SPEED);
								rightMotor.forward();
								leftMotor.forward();
							}

							// ROBOT IS TOO FAR FROM THE WALL - INCREASE OUTSIDE
							// WHEEL, DECREASE
							// INSIDE WHEEL
							if (bang_bang_error > BAND_WIDTH) {
								rightMotor.setSpeed(motorLow + LOW_CONSTANT);
								leftMotor.setSpeed(motorHigh);
								rightMotor.forward();
								leftMotor.forward();
							}

							// ROBOT IS TOO FAR FROM THE WALL - DECREASE OUTSIDE
							// WHEEL, INCREASE
							// INSIDE WHEEL
							if (bang_bang_error < BAND_WIDTH) {
								rightMotor.setSpeed(FORWARD_SPEED + delta);
								leftMotor.setSpeed(FORWARD_SPEED - delta * TIMES_TWO);
								rightMotor.forward();
								leftMotor.forward();
							}

							// IF THE ROBOT HAS SAFELY "HALF-CONTOURED" THE
							// WALL, BREAK OUT OF LOOP
							if (isSafe(orientation)) {
								// ROTATE SENSOR BACK TO ORIGINAL POSITION
								usMotor.rotateTo((int) SENSOR_ROTATE_FROM_WALL);
								// STOP MOTORS
								leftMotor.setSpeed(ZERO);
								rightMotor.setSpeed(ZERO);
								// BREAK OUT OF BANG BANG CONTROLLER
								break wallfollower_loop;
							}
						}

						// BREAK OUT OF LOOP AND RECALCULATE EVERYTHING BASED ON
						// NEW POSITION
						break moving_loop;
					}
				}
			} else {
				// ROTATE TO DESIRED DESTINATION
				leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, distance), true);
				rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, distance), false);
			}

			try {
				// SMOOTHED OUT OUR MOVEMENT
				Thread.sleep(SMALL_SLEEP);
			} catch (InterruptedException e) {
			}

			// RECALCULATE NEW READINGS FROM ODOMETER
			currentX = odometer.getX();
			currentY = odometer.getY();
			currentTheta = odometer.getTheta();

		}

		// STOP MOTORS
		isNavigating = false;
		leftMotor.setSpeed(ZERO);
		rightMotor.setSpeed(ZERO);
	}

	public void turnTo(double theta, double ctheta) {
		// CALCULATE DESIRED ANGLE (ORIENTATION - CURRENT READING FROM ODOMETER)
		changeTheta = theta - ctheta;

		// ENSURES WE ARE ROTATING BY THE SMALLEST AGLE
		if (changeTheta < -Math.PI) {
			changeTheta += TIMES_TWO * Math.PI;
		} else if (changeTheta > Math.PI) {
			changeTheta -= TIMES_TWO * Math.PI;
		}

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// ROTATE WHEELS BY DESIRED ANGLE
		leftMotor.rotate(convertAngle(Lab5.WHEEL_RADIUS, track, changeTheta), true);
		rightMotor.rotate(-convertAngle(Lab5.WHEEL_RADIUS, track, changeTheta), false);
		currentTheta = odometer.getTheta();

	}

	// IMPLEMENTED THE METHOD AS ASKED IN THE INSTRUCTIONS, WE DON'T REALLY EVER
	// USE IT BECAUSE WE FOUND IT MUCH MORE CONVENIENT TO SIMPLY USE THE ISMOVING METHOD
	// PROVIDED BY THE LEJOS API 
	private boolean isNavigating() {
		return isNavigating;
	}

	// CONVERT DISTANCE, TAKEN FROM SQUAREDRIVER CLASS IN LAB 2
	public int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// CHECK IF THE ROBOT IS MOVING
	private boolean isMoving() {
		return rightMotor.isMoving() || leftMotor.isMoving();
	}

	// CHECK IF OBSTACLE HAS BEEN AVOIDED AND IT IS SAFE TO GO BACK ON TRACK
	private boolean isSafe(double orientation) {
		return Math.abs((orientation + RIGHT_ANGLE) - odometer.getTheta()) < SAFETY_DEGREE_ERROR;
	}

	// CONVERT ANGLE, TAKEN FROM SQUAREDRIVER CLASS IN LAB 2
	public int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, width * angle / 2);
	}

}
