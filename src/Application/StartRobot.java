package Application;

import java.io.IOException;
import java.util.HashMap;

import Localization.LightLocalizer;
import Localization.USLocalizer;
import Navigation.Navigation;
import Odometer.LCDInfo;
import Odometer.Odometer;
import SensorData.USPoller;
import Wifi.WifiConnection;
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

//LAB 5 OBJECT SEARCHING
//WEIPING REN - 260613810
//ALEXANDER BRATYSHKIN - 260684228

public class StartRobot {

	/**
	 * Instantiation of all motors
	 */
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
	private static final EV3ColorSensor lightSensorClaw = new EV3ColorSensor(LocalEV3.get().getPort("S3"));

	private static final String SERVER_IP = "192.168.2.10";
	private static final int TEAM_NUMBER = 1;

	/**
	 * Hashmap Holding All Received Data
	 */
	static HashMap<String, Integer> text;
	
	private static int LGZy, LGZx, CSC, BSC, CTN, BTN, URZx, LRZy, LRZx, URZy, UGZy, UGZx;

	// private static TextLCD LCD = LocalEV3.get().getTextLCD();

	public static void main(String[] args) {

		// INITIALIZE HIGH SENSOR
		SampleProvider usValueHigh = usSensorHigh.getMode("Distance");
		float[] usDataHigh = new float[usValueHigh.sampleSize()];

		SampleProvider usValueLow = usSensorLow.getMode("Distance");
		float[] usDataLow = new float[usValueLow.sampleSize()];

		final TextLCD t = LocalEV3.get().getTextLCD();

		USLocalizer.LocalizationType type = USLocalizer.LocalizationType.FALLING_EDGE;

		Odometer odometer = new Odometer(leftMotor, rightMotor, 50, true);

		USPoller usPollerHigh = new USPoller(usValueHigh, usDataHigh);
		USPoller usPollerLow = new USPoller(usValueLow, usDataLow);

		// INITIALIZE COLOUR SENSOR, FIRST IN RGB MODE
		//// EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
		// colorSensor.setFloodlight(lejos.robotics.Color.WHITE);
		// SampleProvider colorValue = colorSensor.getMode("RGB");
		// float[] colorData = new float[colorValue.sampleSize()];
		// LSPoller lsPoller = new LSPoller(colorValue, colorData);

		ObjectDetector objectDetect = new ObjectDetector(usPollerHigh, usPollerLow, t);

		Navigation navigator = new Navigation(odometer, usPollerHigh);

		int buttonChoice;

		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left    | Right >", 0, 0);
			t.drawString("          |        ", 0, 1);
			t.drawString(" PART 1   |   PART 2", 0, 2);
			t.drawString("       	| 	 ", 0, 3);
			t.drawString("          |       ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			// usPollerHigh.start();
			// new Thread(usPollerLow).start();
			// objectDetect.start();
			t.clear();
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
				}
			}

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
			

		} else {

			// pulleyMotor.rotate(90);
			t.clear();
			LCDInfo lcd = new LCDInfo(odometer);

			// while(true){
			// t.drawString("TACHO: " + pulleyMotor.getTachoCount(), 0, 3);
			//
			// }

			// pulleyMotor.rotate(-3000);

			// DO US LOCALIZATION
			USLocalizer usl = new USLocalizer(odometer, usValueLow, usDataLow, type);
			usl.doLocalization(navigator);

			// SWITCH TO RED MODE FOR LIGHT LOCALIZATION
			SampleProvider colorValueLoc = lightSensorBottom.getMode("Red");
			float[] colorDataLoc = new float[colorValueLoc.sampleSize()];

			// DO LIGHT LOCALIZATION
			LightLocalizer lsl = new LightLocalizer(odometer, colorValueLoc, colorDataLoc);
			lsl.doLocalization(navigator);

			while (Button.waitForAnyPress() != Button.ID_ESCAPE)
				;

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);

	}

	public int getLGZy() {
		return LGZy;
	}

	public static void setLGZy(int lGZy) {
		LGZy = lGZy;
	}

	public int getLGZx() {
		return LGZx;
	}

	public static void setLGZx(int lGZx) {
		LGZx = lGZx;
	}

	public int getCSC() {
		return CSC;
	}

	public static void setCSC(int cSC) {
		CSC = cSC;
	}

	public int getBSC() {
		return BSC;
	}

	public static void setBSC(int bSC) {
		BSC = bSC;
	}

	public int getCTN() {
		return CTN;
	}

	public static void setCTN(int cTN) {
		CTN = cTN;
	}

	public int getBTN() {
		return BTN;
	}

	public static void setBTN(int bTN) {
		BTN = bTN;
	}

	public int getURZx() {
		return URZx;
	}

	public static void setURZx(int uRZx) {
		URZx = uRZx;
	}

	public int getLRZy() {
		return LRZy;
	}

	public static void setLRZy(int lRZy) {
		LRZy = lRZy;
	}

	public int getLRZx() {
		return LRZx;
	}

	public static void setLRZx(int lRZx) {
		LRZx = lRZx;
	}

	public int getURZy() {
		return URZy;
	}

	public static void setURZy(int uRZy) {
		URZy = uRZy;
	}

	public int getUGZy() {
		return UGZy;
	}

	public static void setUGZy(int uGZy) {
		UGZy = uGZy;
	}

	public int getUGZx() {
		return UGZx;
	}

	public static void setUGZx(int uGZx) {
		UGZx = uGZx;
	}

}
