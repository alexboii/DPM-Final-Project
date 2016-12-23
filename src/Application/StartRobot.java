package Application;

import java.io.IOException;
import java.util.HashMap;

import Localization.LightLocalizer;
import Localization.USLocalizer;
import Navigation.Navigation;
import Odometer.LCDInfo;
import Odometer.Odometer;
import SensorData.LSPoller;
import SensorData.USPoller;
import Wifi.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This is the class which contains the main() method, therefore it is from here
 * that we are going to execute all subsequent classes and methods needed to run
 * our robot. At first, the robot instantiates all platforms and hardware parts
 * of our robot, i.e. the two ultrasonic sensors with their respective pollers,
 * the motors for the wheels, the claw and the pulley, and finally the light
 * sensor. The display of the screen is also initialized from this class.
 * Threads are started for the pollers of both ultrasonic sensors and for the
 * light sensors. The robot then waits for the user to either press the left
 * button or the right button. The left button is clicked whenever we enter
 * competition mode, that is, we tell the robot to await for the retrieval of
 * parameters from the competition's server. Afterwards, the robot calls the
 * setters for the parameters that we have just retrieved, and assigns these
 * parameters based on the respective role of the robot, i.e. green zone is the
 * drop-off zone and red zone is the forbidden zone for the tower builder, and
 * vice versa for the garbage collector. After this, the robot starts the
 * ultrasonic localization. Once that is performed, the robot starts the light
 * localization. Finally, the most important thread of the class is executed,
 * which is the RobotMovement's thread which is the controller of the robot's
 * actions. Through all of this, the robot begins to record the EV3's internal
 * time in milliseconds and awaits until 4:30 minutes pass, after which it
 * simply travels to the drop-off zone no matter in which situation the robot
 * is, and finally goes back to its starting corner.
 * 
 * The option to select a right button was implemented to facilitate all testing
 * and experiments that did not require any Wifi transmission.
 * 
 * @author Alexander Bratyshkin and Sebastian Andarde
 *
 */
public class StartRobot {

	/**
	 * Instantiation of all motors
	 */

	// MOTORS ARE REVERSED FOR LEFT AND RIGHT
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor clawMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor pulleyMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/**
	 * Instantiation of all sensors
	 */
	private static final SensorModes usSensorHigh = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	private static final SensorModes usSensorLow = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	private static final EV3ColorSensor lightSensorBottom = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

	/**
	 * Server IP's and team number, used for retrieval of parameters
	 */
	private static final String SERVER_IP = "192.168.2.3";
	private static final double TEAM_NUMBER = 14;

	/**
	 * Hashmap holding all received data
	 */
	static HashMap<String, Integer> text;

	/**
	 * Define all parameters to be received from the WifiConnection class
	 */
	public static double LGZy, LGZx, CSC, BSC, CTN, BTN, URZx, LRZy, LRZx, URZy, UGZy, UGZx;

	/**
	 * Define all parameters related to the forbidden zones and drop-off zones
	 */
	public static double LFZy, LFZx, UFZy, UFZx, LDZy, LDZx, UDZy, UDZx;

	/**
	 * Receive and assign parameters from client. Start the robot
	 * 
	 * @param args
	 */

	/**
	 * Define program constants
	 */
	private static final double TILE = 30.48;

	@SuppressWarnings("unused")
	public static void main(String[] args) {

		// INSTANTIATE HIGH US SENSOR
		SampleProvider usValueHigh = usSensorHigh.getMode("Distance");
		float[] usDataHigh = new float[usValueHigh.sampleSize()];

		// INSTANTIATE LOW US SENSOR
		SampleProvider usValueLow = usSensorLow.getMode("Distance");
		float[] usDataLow = new float[usValueLow.sampleSize()];

		// INSTANTIATE SCREEN DISPLAY
		final TextLCD t = LocalEV3.get().getTextLCD();

		// INSTANTIATE US LOCALIZATION
		USLocalizer.LocalizationType type = USLocalizer.LocalizationType.FALLING_EDGE;

		// INSTANTIATE ODOMETER
		Odometer odometer = new Odometer(leftMotor, rightMotor, 50, true);

		// INSTANTIATE COLOUR SENSOR
		SampleProvider colorValueLoc = lightSensorBottom.getMode("Red");
		float[] colorDataLoc = new float[colorValueLoc.sampleSize()];

		USPoller usPollerHigh = new USPoller(usValueHigh, usDataHigh);
		USPoller usPollerLow = new USPoller(usValueLow, usDataLow);

		// INSTANTIATE NAVIGATOR
		Navigation navigator = new Navigation(odometer, usPollerLow, usPollerHigh);

		// INSTANTIATE LIGHT SENSOR POLLER
		LSPoller lsPoller = new LSPoller(colorValueLoc, colorDataLoc);

		// START ALL POLLERS, KEPT BOTH US POLLERS ON DIFFERENT THREADS JUST TO
		// MAKE SURE
		lsPoller.start();
		usPollerHigh.start();
		new Thread(usPollerLow).start();

		LCDInfo lcd = new LCDInfo(odometer); // t.clear();

		int buttonChoice;

		do {
			// CLEAR THE DISPLAY
			t.clear();

			// ASK THE USER WHAT OPTION HE WANTS
			t.drawString("<   		 | Testing >", 0, 0);
			t.drawString("Competition  |   ", 0, 1);
			t.drawString(" 		     |  ", 0, 2);
			t.drawString("      	 | 		 ", 0, 3);
			t.drawString("           |       ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
			clawMotor.stop();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			// usPollerHigh.start();
			t.clear();
			WifiConnection conn = null;
			try {
				System.out.println("Connecting...");
				conn = new WifiConnection(SERVER_IP, (int) TEAM_NUMBER, true);
			} catch (IOException e) {
				System.out.println("Connection failed");
			}

			/*
			 * This section of the code reads and prints the data received from
			 * the server, stored as a HashMap with String keys and Integer
			 * values.
			 */
			if (conn != null) {
				text = conn.StartData;
				if (t == null) {
					System.out.println("Failed to read transmission");
				} else {
					System.out.println("Transmission read:\n" + text.toString());

					setCSC(text.get("CSC"));
					setBSC(text.get("BSC"));
					setCTN(text.get("CTN"));
					setBTN(text.get("BTN"));
					setLGZy(text.get("LGZy"));
					setLGZx(text.get("LGZx"));
					setURZx(text.get("URZx"));
					setLRZy(text.get("LRZy"));
					setLRZx(text.get("LRZx"));
					setURZy(text.get("URZy"));
					setUGZy(text.get("UGZy"));
					setUGZx(text.get("UGZx"));

				}
			}

			if (BTN == TEAM_NUMBER) {
				setLFZy(LRZy);
				setLFZx(LRZx);
				setUFZy(URZy);
				setUFZx(URZx);
				setLDZy(LGZy);
				setLDZx(LGZx);
				setUDZy(UGZy);
				setUDZx(UGZx);
			} else {
				setLFZy(LGZy);
				setLFZx(LGZx);
				setUFZy(UGZy);
				setUFZx(UGZx);
				setLDZy(LRZy);
				setLDZx(LRZx);
				setUDZy(URZy);
				setUDZx(URZx);
			}

			long currentTime = System.currentTimeMillis();

			// // DO US LOCALIZATION
			USLocalizer usl = new USLocalizer(odometer, usPollerLow, type);
			usl.doLocalization(navigator);

			// DO LIGHT LOCALIZATION
			LightLocalizer lsl = new LightLocalizer(odometer, colorValueLoc, colorDataLoc);
			lsl.doLocalization(navigator);

			// BEEP TO LET EVERYONE KNOW IT HAS FINISHED LOCALIZING
			Sound.beepSequence();

			RobotMovement attempt = new RobotMovement(odometer, navigator, usPollerLow, usPollerHigh, clawMotor,
					pulleyMotor, colorValueLoc, colorDataLoc);
			attempt.start();

			// RUN TIMER FOR 4:30 MINS
			while (System.currentTimeMillis() < (currentTime + 270000)) {

			}

			// ABANDON WHATEVER YOU ARE DOING,
			attempt.goToDropOffZone();
			navigator.travelTo(0, 0, true);

		} else {

			// TESTING ZONE

		}

	}

	/**
	 * @return y coordinate of upper right corner of Green Zone
	 */
	public double getLGZy() {
		return LGZy;
	}

	/**
	 * Set y coordinate of upper right corner of Green Zone
	 * 
	 * @param y
	 *            coordinate of upper right corner of Green Zone
	 */
	public static void setLGZy(double lGZy) {
		LGZy = (lGZy * TILE);
	}

	/**
	 * @return x coordinate of upper right corner of Green Zone
	 */
	public double getLGZx() {
		return LGZx;
	}

	/**
	 * Set x coordinate of upper right corner of Green Zone
	 * 
	 * @param x
	 *            coordinate of upper right corner of Green Zone
	 */
	public static void setLGZx(double lGZx) {
		LGZx = (lGZx * -TILE);
	}

	/**
	 * @return Collecting start corner
	 */
	public double getCSC() {
		return CSC;
	}

	/**
	 * Set collecting start corner
	 * 
	 * @param cSC
	 */
	public static void setCSC(double cSC) {
		CSC = cSC;
	}

	/**
	 * @return Builder start corner
	 */
	public double getBSC() {
		return BSC;
	}

	/**
	 * Set builder start corner
	 * 
	 * @param bSC
	 */
	public static void setBSC(double bSC) {
		BSC = bSC;
	}

	/**
	 * @return Collector team number
	 */
	public double getCTN() {
		return CTN;
	}

	/**
	 * Set collector team number
	 * 
	 * @param cTN
	 */
	public static void setCTN(double cTN) {
		CTN = cTN;
	}

	/**
	 * @return Builder team number
	 */
	public double getBTN() {
		return BTN;
	}

	/**
	 * Set builder team number
	 * 
	 * @param bTN
	 */
	public static void setBTN(double bTN) {
		BTN = bTN;
	}

	/**
	 * @return x coordinate of upper right corner of Red Zone
	 */
	public double getURZx() {
		return URZx;
	}

	/**
	 * Set x coordinate of upper right corner of Red Zone
	 * 
	 * @param uRZx
	 */
	public static void setURZx(double uRZx) {
		URZx = -TILE * uRZx;
	}

	/**
	 * @return y coordinate of lower left corner of Red Zone
	 */
	public double getLRZy() {
		return LRZy;
	}

	/**
	 * Set y coordinate of lower left corner of Red Zone
	 * 
	 * @param lRZy
	 */
	public static void setLRZy(double lRZy) {
		LRZy = TILE * lRZy;
	}

	/**
	 * @return x coordinate of lower left corner of Red Zone
	 */
	public double getLRZx() {
		return LRZx;
	}

	/**
	 * Set x coordinate of lower left corner of Red Zone
	 * 
	 * @param lRZx
	 */
	public static void setLRZx(double lRZx) {
		LRZx = lRZx * -TILE;
	}

	/**
	 * @return y coordinate of upper right corner of Red Zone
	 */
	public double getURZy() {
		return URZy;
	}

	/**
	 * Set y coordinate of upper right corner of Red Zone
	 * 
	 * @param uRZy
	 */
	public static void setURZy(double uRZy) {
		URZy = uRZy * TILE;
	}

	/**
	 * @return y coordinate of upper right corner of Green Zone
	 */
	public double getUGZy() {
		return UGZy;
	}

	/**
	 * Set y coordinate of upper right corner of Green Zone
	 * 
	 * @param uGZy
	 */
	public static void setUGZy(double uGZy) {
		UGZy = uGZy * TILE;
	}

	/**
	 * @return x coordinate of upper right corner of Green Zone
	 */
	public double getUGZx() {
		return UGZx;
	}

	/**
	 * Set x coordinate of upper right corner of Green Zone
	 * 
	 * @param uGZx
	 */
	public static void setUGZx(double uGZx) {
		UGZx = (uGZx * -TILE);
	}

	/**
	 * @return the y coordinate of the lower forbidden zone
	 */
	public static double getLFZy() {
		return LFZy;
	}

	/**
	 * Set the y coordinate of the lower forbidden zone
	 * 
	 * @param lFZy
	 *            the lFZy to set
	 */
	public static void setLFZy(double lFZy) {
		LFZy = lFZy;
	}

	/**
	 * 
	 * @return the x coordinate of the lower forbidden zone
	 */
	public static double getLFZx() {
		return LFZx;
	}

	/**
	 * Set the x coordinate of the lower forbidden zone
	 * 
	 * @param lFZx
	 *            the lFZx to set
	 */
	public static void setLFZx(double lFZx) {
		LFZx = lFZx;
	}

	/**
	 * @return The y coordinate of the upper forbidden zone
	 */
	public static double getUFZy() {
		return UFZy;
	}

	/**
	 * Set the y coordinate of the upper forbidden zone
	 * 
	 * @param uFZy
	 * 
	 */
	public static void setUFZy(double uFZy) {
		UFZy = uFZy;
	}

	/**
	 * @return the x coordinate of the upper forbidden zone
	 */
	public static double getUFZx() {
		return UFZx;
	}

	/**
	 * Set the x coordinate of the upper forbidden zone
	 * 
	 * @param uFZx
	 *            the uFZx to set
	 */
	public static void setUFZx(double uFZx) {
		UFZx = uFZx;
	}

	/**
	 * @return the Y coordinate of the lower drop-off zone
	 * 
	 */
	public static double getLDZy() {
		return LDZy;
	}

	/**
	 * Set the Y coordinate of the lower drop-off zone
	 * 
	 * @param lDZy
	 *            the lDZy to set
	 */
	public static void setLDZy(double lDZy) {
		LDZy = lDZy;
	}

	/**
	 * @return the X coordinate of the lower drop-off zone
	 */
	public static double getLDZx() {
		return LDZx;
	}

	/**
	 * Set the X coordinate of the lower drop-off zone
	 * 
	 * @param LDZx
	 *            the LDZx to set
	 */
	public static void setLDZx(double lDZx) {
		LDZx = lDZx;
	}

	/**
	 * @return the Y coordinate of the upper drop-off zone
	 */
	public static double getUDZy() {
		return UDZy;
	}

	/**
	 * Set the Y coordinate of the upper drop-off zone
	 * 
	 * @param uDZy
	 *            the uDZy to set
	 */
	public static void setUDZy(double uDZy) {
		UDZy = uDZy;
	}

	/**
	 * @return the X coordinate of the upper drop-off zone
	 */
	public static double getUDZx() {
		return UDZx;
	}

	/**
	 * Set the X coordinate of the upper drop-off zone
	 * 
	 * @param uDZx
	 *            the uDZx to set
	 */
	public static void setUDZx(double uDZx) {
		UDZx = uDZx;
	}

}
