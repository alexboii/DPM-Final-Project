package Application;
import Navigation.Navigation;
import Odometer.Odometer;
import SensorData.LSPoller;
import SensorData.USPoller;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class RobotMovement extends Thread {

	Odometer odometer;
	Navigation navigator;
	USPoller usPoller;
	LSPoller lsPoller;
	EV3LargeRegulatedMotor usMotor;
	boolean blue_found = false;
	public static final int ROTATE_SPEED = 50;
	private static final int BAND_WIDTH = 15;

	private static final int TURN_ANGLE_1 = -20;
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


	public RobotMovement(Odometer odometer, Navigation navigator, USPoller usPoller, LSPoller lsPoller,
			EV3LargeRegulatedMotor usMotor) {
		this.navigator = navigator;
		this.odometer = odometer;
		this.usPoller = usPoller;
		this.lsPoller = lsPoller;
		this.usMotor = usMotor;
	}

	public void run() {

		int distance_counter = 0;
		boolean already_have_blue = false;
		navigator.turnTo(TURN_ANGLE_1, false);

		// WHILE WE HAVEN'T FOUND A BLUE OBJECT
		while (!blue_found) {

			// SCAN THE LOCATION
			navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

			// IF ROBOT FINDS AN OBJECT WITHIN THRESHOLD DISTANCE
			if (usPoller.getDistance() < OBJECT_DISTANCE) {

				// COUNTER DEFINED TO FILTER OUT FALSE NEGATIVES
				distance_counter++;

				if (distance_counter > DISTANCE_COUNTER_LIMIT) {

					distance_counter = 0;

					// STOP MOTORS
					navigator.setSpeeds(0, 0);

					// ADVANCE TOWARDS THE NEW FOUND OBJECT
					double go_by = (usPoller.getDistance() - DISTANCE_OFFSET > 0
							? (usPoller.getDistance() - DISTANCE_OFFSET) : FORWARD_DISTANCE);
					navigator.goForward(go_by, false);

					// STOP MOTORS
					navigator.setSpeeds(0, 0);

					// IF THIS IS THE FIRST BLUE OBJECT WE HAVE FOUND
					if (lsPoller.isBlue() && !already_have_blue) {
						Sound.beep();
						navigator.goForward(FORWARD_DISTANCE, false);
						navigator.setSpeeds(0, 0);
						already_have_blue = true;

						navigator.travelTo(WAYPOINT_X, WAYPOINT_Y, true);
						Sound.beep();
						Sound.beep();
						Sound.beep();
						break;
					} else {

						// IF IT'S A WOODEN BLOCK, THEN ENTER BANG BANG MODE

						Sound.beep();
						Sound.beep();

						// MAKE A SHARP 90 DEGREE TURN AND ROTATE SENSOR
						navigator.turnTo(odometer.getTheta() + RIGHT_ANGLE, true);
						usMotor.rotateTo(SENSOR_ROTATE);

						double lastTheta = odometer.getTheta();
						wallfollower_loop: while (true) {

							int bang_bang_error = (int) (usPoller.getDistance() - 3);

							// ROBOT IS AT THE RIGHT DISTANCE FROM WALL, KEEP
							// GOING
							if (Math.abs(bang_bang_error) <= BAND_WIDTH) {
								navigator.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
							}

							// ROBOT IS TOO FAR FROM THE WALL - INCREASE OUTSIDE
							// WHEEL, DECREASE
							// INSIDE WHEEL
							if (bang_bang_error > BAND_WIDTH) {
								navigator.setSpeeds(BB_FAST_SPEED, BB_SLOW_SPEED);
							}

							// ROBOT IS TOO CLOSE TO WALL - DECRASE OUTISDE
							// INCREASE INSIDE
							if (bang_bang_error < BAND_WIDTH) {
								navigator.setSpeeds(BB_SLOW_SPEED, BB_FAST_SPEED);
							}

							// IF THE ROBOT HAS SAFELY "HALF-CONTOURED" THE
							// WALL, BREAK OUT OF LOOP
							if (Math.abs(odometer.getTheta() - lastTheta) > ANGLE_LIMIT_BB) {
								// ROTATE SENSOR BACK TO ORIGINAL POSITION
								// STOP MOTORS
								navigator.setSpeeds(0, 0);
								usMotor.rotateTo(0);
								break wallfollower_loop;
							}
						}

					}
				}

			}

			// IF NOTHING WAS FOUND WITHIN THE SCAN, GO FORWARD BY
			// FORWARD_DISTANCE AND REPEAT THE SCAN
			if (odometer.getTheta() > ANGLE_LIMIT) {
				navigator.setSpeeds(0, 0);
				navigator.turnTo(TURN_ANGLE_2, true);
				navigator.goForward(FORWARD_DISTANCE, false);
				navigator.turnTo(TURN_ANGLE_3, false);
			}
		}

		navigator.setSpeeds(0, 0);
	}

}
