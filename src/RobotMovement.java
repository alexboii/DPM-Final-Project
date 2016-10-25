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

	private final int delta = 175;
	private final int LOW_CONSTANT = 15;
	private static final int motorLow = 100;
	private static final int motorHigh = 260;

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
		navigator.turnTo(65, false);

		while (!blue_found) {
			// distance_counter = 0;
			navigator.setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
			// System.out.println("\n\n\n\nUS: " + usPoller.getDistance());
			if (usPoller.getDistance() < 25) {
				distance_counter++;

				if (distance_counter > 20) {
					LCD.drawString("US: " + usPoller.getDistance(), 0, 4);
					navigator.setSpeeds(0, 0);

					double go_by = (usPoller.getDistance() - 8 > 0 ? (usPoller.getDistance() - 8) : 10);
					navigator.goForward(go_by);
					navigator.setSpeeds(0, 0);

					if (lsPoller.isBlue() && !already_have_blue) {
						// navigator.setSpeeds(0, 0);
						navigator.setSpeeds(30, 30);
						navigator.goForward(10);
						navigator.setSpeeds(0, 0);
						already_have_blue = true;
						
						int NEW_X = (int) (-60 - odometer.getX());
						int NEW_Y = (int) (60 - odometer.getY());
//						int NEW_X = (int) (odometer.getX());
//						int NEW_Y = (int) (odometer.getY());
//						int NEW_THETA = (int) (odometer.getTheta());
						System.out.println("\n\n\n\n X: " + NEW_X);
						System.out.println("\n\n\n\n\n Y: " + NEW_Y);
//						navigator.turnTo(90, true);
						int[][] NEW_WAYPOINTS = {{NEW_X,NEW_Y}};
						Lab5.WAYPOINTS_DEMO_2 = NEW_WAYPOINTS;
//						Lab5.NEW_X = NEW_X;
//						Lab5.NEW_Y = NEW_Y;
//						Lab5.NEW_THETA = NEW_THETA;
						Lab5.start = true;
						break;
					} else {
						navigator.turnTo(odometer.getTheta() + 90, true);
						usMotor.rotateTo(80);
						double lastTheta = odometer.getTheta();
						wallfollower_loop: while (true) {

							int bang_bang_error = (int) (usPoller.getDistance() - 3);

							// ROBOT IS AT THE RIGHT DISTANCE FROM WALL, KEEP
							// GOING
							if (Math.abs(bang_bang_error) <= BAND_WIDTH) {
								navigator.setSpeeds(200, 200);
							}

							// ROBOT IS TOO FAR FROM THE WALL - INCREASE OUTSIDE
							// WHEEL, DECREASE
							// INSIDE WHEEL
							if (bang_bang_error > BAND_WIDTH) {
								navigator.setSpeeds(260, 115);
							}

							// ROBOT IS TOO FAR FROM THE WALL - DECREASE OUTSIDE
							// WHEEL, INCREASE
							// INSIDE WHEEL
							if (bang_bang_error < BAND_WIDTH) {
								navigator.setSpeeds(115, 260);
							}

							// IF THE ROBOT HAS SAFELY "HALF-CONTOURED" THE
							// WALL, BREAK OUT OF LOOP
							if (Math.abs(odometer.getTheta() - lastTheta) > 110) {
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

			if (odometer.getTheta() > 200) {
				navigator.setSpeeds(0, 0);
				navigator.turnTo(135, true);
				navigator.goForward(10);
				navigator.turnTo(55, false);
			}
		}

		navigator.setSpeeds(0, 0);
	}

}
