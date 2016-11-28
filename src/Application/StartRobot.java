package Application;

import java.io.IOException;
import java.util.HashMap;

import Localization.LightLocalizer;
import Localization.USLocalizer;
import Navigation.Navigation;
import Odometer.LCDInfo;
import Odometer.Odometer;
import Odometer.OdometerCorrection;
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
 * our robot.
 * 
 * @author Alex
 * @author Seb
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
	//                                                               private static final EV3ColorSensor lightSensorBack = new EV3ColorSensor(LocalEV3.get().getPort("S3"));

	// private static final EV3ColorSensor lightSensorClaw = new
	// EV3ColorSensor(LocalEV3.get().getPort("S3"));

	/**
	 * Server IP's and team number, used for retrieval of parameters
	 */
	private static final String SERVER_IP = "192.168.2.3";
	private static final int TEAM_NUMBER = 14;	

	/**
	 * Hashmap holding all received data
	 */
	static HashMap<String, Integer> text;

	/**
	 * Define all parameters to be received from the WifiConnection class
	 */
	public static int LGZy, LGZx, CSC, BSC, CTN, BTN, URZx, LRZy, LRZx, URZy, UGZy, UGZx;
	
	/**
	 * Define all parameters related to the forbidden zones and drop-off zones 
	 */
	public static int LFZy, LFZx, UFZy, UFZx, LDZy, LDZx, UDZy, UDZx;

	/**
	 * Receive and assign parameters from client. Start the robot
	 * 
	 * @param args
	 */

	/**
	 * Define program constants
	 */
	private static final double TILE = 30.48;

	private static final int CLAW_ANGLE = 10;
	private static final int FULL_CIRCLE = 360;

	public static void main(String[] args) {

		// INITIALIZE HIGH SENSOR
		SampleProvider usValueHigh = usSensorHigh.getMode("Distance");
		float[] usDataHigh = new float[usValueHigh.sampleSize()];

		SampleProvider usValueLow = usSensorLow.getMode("Distance");
		float[] usDataLow = new float[usValueLow.sampleSize()];

		final TextLCD t = LocalEV3.get().getTextLCD();

		USLocalizer.LocalizationType type = USLocalizer.LocalizationType.FALLING_EDGE;

		Odometer odometer = new Odometer(leftMotor, rightMotor, 50, true);
		//OdometerCorrection odoCor = new OdometerCorrection(odometer);
		
		SampleProvider colorValueLoc = lightSensorBottom.getMode("Red");
		float[] colorDataLoc = new float[colorValueLoc.sampleSize()];

		USPoller usPollerHigh = new USPoller(usValueHigh, usDataHigh);
		USPoller usPollerLow = new USPoller(usValueLow, usDataLow);

		

		Navigation navigator = new Navigation(odometer, usPollerLow, usPollerHigh);
		 LSPoller lsPoller = new LSPoller(colorValueLoc, colorDataLoc);
		
		lsPoller.start();
		usPollerHigh.start();
		new Thread(usPollerLow).start();

		int buttonChoice;

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Stack    | THE  >", 0, 0);
			t.drawString("  Builder  | BOX    ", 0, 1);
			t.drawString(" 		     |  ", 0, 2);
			t.drawString("      	 | 		 ", 0, 3);
			t.drawString("           |       ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
			clawMotor.stop();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			// usPollerHigh.start();
			t.clear();
			LCDInfo lcd = new LCDInfo(odometer, usPollerHigh, usPollerLow, lsPoller); // t.clear();
			WifiConnection conn = null;
			try {
				System.out.println("Connecting...");
				conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, true);
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

					setLGZy(text.get("LGZy"));
					setLGZx(text.get("LGZx"));
					setCSC(text.get("CSC"));
					setBSC(text.get("BSC"));
					setCTN(text.get("CTN"));
					setBTN(text.get("BTN"));
					setURZx(text.get("URZx"));
					setLRZy(text.get("LRZy"));
					setLRZx(text.get("LRZx"));
					setURZy(text.get("URZy"));
					setUGZy(text.get("UGZy"));
					setUGZx(text.get("UGZx"));

				}
			}

			if(BTN == TEAM_NUMBER){
				setLFZy(LRZy);
				setLFZx(LRZx);
				setUFZy(URZy);
				setUFZx(URZx);
				setLDZy(LGZy);
				setLDZx(LGZx);
				setUDZy(UGZy);
				setUDZx(UGZx);
			}else{
				setLFZy(LGZy);
				setLFZx(LGZx);
				setUFZy(UGZy);
				setUFZx(UGZx);
				setLDZy(LRZy);
				setLDZx(LRZx);
				setUDZy(URZy);
				setUDZx(URZx);
			}

			// // DO US LOCALIZATION
			USLocalizer usl = new USLocalizer(odometer, usPollerLow, type);
			usl.doLocalization(navigator);


			// DO LIGHT LOCALIZATION
			LightLocalizer lsl = new LightLocalizer(odometer, colorValueLoc, colorDataLoc);
			lsl.doLocalization(navigator);

			RobotMovement attempt = new RobotMovement(odometer, navigator, usPollerLow, usPollerHigh, clawMotor,
					pulleyMotor);
			attempt.start();

		} else {

			odometer.setPosition(new double[] { 0, 0, 90 }, new boolean[] { true, true, true });

			
			t.clear();
			LCDInfo lcd = new LCDInfo(odometer, usPollerHigh, usPollerLow, lsPoller);

			setLDZy(3);
			setUDZy(4);

			
			
			setLDZx(2);
			setUDZx(3);

			RobotMovement attempt = new RobotMovement(odometer, navigator, usPollerLow, usPollerHigh, clawMotor,
					pulleyMotor);
			attempt.start();


		}

	}

	public static int findSmallestDistance(double[] distances) {
		int index;
		double min;

		index = 0;
		min = distances[0];

		for (int i = 0; i < distances.length; ++i) {
			if (distances[i] < min) {
				min = distances[i];
				index = i;
			}
		}

		return index;
	}



	public static void openClaw() {
		clawMotor.rotate(CLAW_ANGLE, false);
	}

	public static void closeClaw() {
		clawMotor.rotate(-CLAW_ANGLE, false);
	}

	// TODO
	// the info below is outdated since we improved the crane design, gotta
	// re-take these measurements

	// 360 = 5 circles = 2.5cm aprox
	// maximum height = 27 circles = 13.5 cm
	// each block = 3cm => maxBlock = 13.5/3 = 4

	public static void pulleyUp(double distance) {
		pulleyMotor.rotate(-(int) (FULL_CIRCLE * distance), false);
	}

	public static void pulleyDown(double distance) {
		pulleyMotor.rotate((int) (FULL_CIRCLE * distance), false);
	}

	/**
	 * @return y coordinate of upper right corner of Green Zone
	 */
	public int getLGZy() {
		return LGZy;
	}

	/**
	 * Set y coordinate of upper right corner of Green Zone
	 * 
	 * @param y
	 *            coordinate of upper right corner of Green Zone
	 */
	public static void setLGZy(int lGZy) {
		LGZy = (int)(lGZy * TILE);
	}

	/**
	 * @return x coordinate of upper right corner of Green Zone
	 */
	public int getLGZx() {
		return LGZx;
	}

	/**
	 * Set x coordinate of upper right corner of Green Zone
	 * 
	 * @param x
	 *            coordinate of upper right corner of Green Zone
	 */
	public static void setLGZx(int lGZx) {
		LGZx = -(int)(lGZx * TILE);
	}

	/**
	 * @return Collecting start corner
	 */
	public int getCSC() {
		return CSC;
	}

	/**
	 * Set collecting start corner
	 * 
	 * @param cSC
	 */
	public static void setCSC(int cSC) {
		CSC = cSC;
	}

	/**
	 * @return Builder start corner
	 */
	public int getBSC() {
		return BSC;
	}

	/**
	 * Set builder start corner
	 * 
	 * @param bSC
	 */
	public static void setBSC(int bSC) {
		BSC = bSC;
	}

	/**
	 * @return Collector team number
	 */
	public int getCTN() {
		return CTN;
	}

	/**
	 * Set collector team number
	 * 
	 * @param cTN
	 */
	public static void setCTN(int cTN) {
		CTN = cTN;
	}

	/**
	 * @return Builder team number
	 */
	public int getBTN() {
		return BTN;
	}

	/**
	 * Set builder team number
	 * 
	 * @param bTN
	 */
	public static void setBTN(int bTN) {
		BTN = bTN;
	}

	/**
	 * @return x coordinate of upper right corner of Red Zone
	 */
	public int getURZx() {
		return URZx;
	}

	/**
	 * Set x coordinate of upper right corner of Red Zone
	 * 
	 * @param uRZx
	 */
	public static void setURZx(int uRZx) {
		URZx = uRZx;
	}

	/**
	 * @return y coordinate of lower left corner of Red Zone
	 */
	public int getLRZy() {
		return LRZy;
	}

	/**
	 * Set y coordinate of lower left corner of Red Zone
	 * 
	 * @param lRZy
	 */
	public static void setLRZy(int lRZy) {
		LRZy = lRZy;
	}

	/**
	 * @return x coordinate of lower left corner of Red Zone
	 */
	public int getLRZx() {
		return LRZx;
	}

	/**
	 * Set x coordinate of lower left corner of Red Zone
	 * 
	 * @param lRZx
	 */
	public static void setLRZx(int lRZx) {
		LRZx = lRZx;
	}

	/**
	 * @return y coordinate of upper right corner of Red Zone
	 */
	public int getURZy() {
		return URZy;
	}

	/**
	 * Set y coordinate of upper right corner of Red Zone
	 * 
	 * @param uRZy
	 */
	public static void setURZy(int uRZy) {
		URZy = uRZy;
	}

	/**
	 * @return y coordinate of upper right corner of Green Zone
	 */
	public int getUGZy() {
		return UGZy;
	}

	/**
	 * Set y coordinate of upper right corner of Green Zone
	 * 
	 * @param uGZy
	 */
	public static void setUGZy(int uGZy) {
		UGZy = (int) (uGZy * TILE);
	}

	/**
	 * @return x coordinate of upper right corner of Green Zone
	 */
	public int getUGZx() {
		return UGZx;
	}

	/**
	 * Set x coordinate of upper right corner of Green Zone
	 * 
	 * @param uGZx
	 */
	public static void setUGZx(int uGZx) {
		UGZx = (int) (uGZx * TILE);
	}
	
	/**
	 * @return the lFZy
	 */
	public static int getLFZy() {
		return LFZy;
	}

	/**
	 * @param lFZy the lFZy to set
	 */
	public static void setLFZy(int lFZy) {
		LFZy = lFZy;
	}

	/**
	 * @return the lFZx
	 */
	public static int getLFZx() {
		return LFZx;
	}

	/**
	 * @param lFZx the lFZx to set
	 */
	public static void setLFZx(int lFZx) {
		LFZx = lFZx;
	}

	/**
	 * @return the uFZy
	 */
	public static int getUFZy() {
		return UFZy;
	}

	/**
	 * @param uFZy the uFZy to set
	 */
	public static void setUFZy(int uFZy) {
		UFZy = uFZy;
	}

	/**
	 * @return the uFZx
	 */
	public static int getUFZx() {
		return UFZx;
	}

	/**
	 * @param uFZx the uFZx to set
	 */
	public static void setUFZx(int uFZx) {
		UFZx = uFZx;
	}

	/**
	 * @return the lDZy
	 */
	public static int getLDZy() {
		return LDZy;
	}

	/**
	 * @param lDZy the lDZy to set
	 */
	public static void setLDZy(int lDZy) {
		LDZy = lDZy;
	}

	/**
	 * @return the LDZx
	 */
	public static int getLDZx() {
		return LDZx;
	}

	/**
	 * @param LDZx the LDZx to set
	 */
	public static void setLDZx(int lDZx) {
		LDZx = lDZx;
	}

	/**
	 * @return the uDZy
	 */
	public static int getUDZy() {
		return UDZy;
	}

	/**
	 * @param uDZy the uDZy to set
	 */
	public static void setUDZy(int uDZy) {
		UDZy = uDZy;
	}

	/**
	 * @return the uDZx
	 */
	public static int getUDZx() {
		return UDZx;
	}

	/**
	 * @param uDZx the uDZx to set
	 */
	public static void setUDZx(int uDZx) {
		UDZx = uDZx;
	}

}
