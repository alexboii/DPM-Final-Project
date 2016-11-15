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
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor clawMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor pulleyMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/**
	 * Instantiation of all sensors
	 */
	private static final SensorModes usSensorHigh = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	private static final SensorModes usSensorLow = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
	private static final EV3ColorSensor lightSensorBottom = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static final EV3ColorSensor lightSensorClaw = new EV3ColorSensor(LocalEV3.get().getPort("S3"));

	/**
	 * Server IP's and team number, used for retrieval of parameters
	 */
	private static final String SERVER_IP = "192.168.2.10";
	private static final int TEAM_NUMBER = 14;

	/**
	 * Hashmap holding all received data
	 */
	static HashMap<String, Integer> text;

	/**
	 * Define all parameters to be received from the WifiConnection class
	 */
	private static int LGZy, LGZx, CSC, BSC, CTN, BTN, URZx, LRZy, LRZx, URZy, UGZy, UGZx;

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

		Odometer odometer = new Odometer(leftMotor, rightMotor);

		USPoller usPollerHigh = new USPoller(usValueHigh, usDataHigh);
		USPoller usPollerLow = new USPoller(usValueLow, usDataLow);
		
		usPollerHigh.start();
		usPollerLow.start();
		odometer.start();
		


		// INITIALIZE COLOUR SENSOR, FIRST IN RGB MODE
		//// EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
		// colorSensor.setFloodlight(lejos.robotics.Color.WHITE);
		// SampleProvider colorValue = colorSensor.getMode("RGB");
		// float[] colorData = new float[colorValue.sampleSize()];
		// LSPoller lsPoller = new LSPoller(colorValue, colorData);

		//ObjectDetector objectDetect = new ObjectDetector(usPollerHigh, usPollerLow, t);

		Navigation navigator = new Navigation(odometer, usPollerHigh);

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
			LCDInfo lcd = new LCDInfo(odometer, usPollerHigh, usPollerLow );
			
			
			
			// while(true){
			// t.drawString("TACHO: " + pulleyMotor.getTachoCount(), 0, 3);
			//
			// }

			// pulleyMotor.rotate(-3000);

//			// DO US LOCALIZATION
//			USLocalizer usl = new USLocalizer(odometer, usValueLow, usDataLow, type);
//			usl.doLocalization(navigator);
//
//			// SWITCH TO RED MODE FOR LIGHT LOCALIZATION
//			SampleProvider colorValueLoc = lightSensorBottom.getMode("Red");
//			float[] colorDataLoc = new float[colorValueLoc.sampleSize()];
//
//			// DO LIGHT LOCALIZATION
//			LightLocalizer lsl = new LightLocalizer(odometer, colorValueLoc, colorDataLoc);
//			lsl.doLocalization(navigator);

			
			odometer.setPosition(new double[] { 0, 0, 0 },
					new boolean[] { true, true, true });
			
			while (Button.waitForAnyPress() != Button.ID_ESCAPE){
				buttonChoice = Button.waitForAnyPress();
				
				
				if(buttonChoice == Button.ID_RIGHT){
					closeClaw();
				}
				
				
				if(buttonChoice == Button.ID_LEFT){
					openClaw();
				}
				
				
				
				if(buttonChoice == Button.ID_UP){
					pulleyUp(1);
				//	navigator.goForward(4);
				}
				
				if(buttonChoice == Button.ID_DOWN){
					pulleyDown(1);
				//	navigator.goForward(-4);

				}
				
				
				
			}
			
			double[] distances = new double[20];
				
				//scan for objects
				//with lower sensor
				
				//save lower sensor for each theta
				
				//go to closest object
				
				//analyze object
				
				//if blue block, grab
				
				//take to -15,-15
				
				
//				for(int i =0 ; i < 20; ++i){
//					navigator.turnTo(i * (Math.PI / 40 ));
//					distances[i]= usPollerLow.getDistance();
//				}
//
//				
//				navigator.turnTo(findSmallestDistance(distances)  * (Math.PI / 40 ) );
//				
//				navigator.goForward(distances[findSmallestDistance(distances)] - 3);
//				
//				if( (usPollerLow.getDistance() + 20) < usPollerHigh.getDistance()){
//					
//					pickUpBlock();
//				}
//				
//				navigator.travelTo(2 * TILE, 0);
//				
				
				
				
			
			
			
			
			
			
			
//			navigator.turnTo(Math.PI/2);
			
			
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);

	}
	
	public static int findSmallestDistance(double[] distances){
		int index;
		double min;
		
		index = 0;
		min = distances[0];
		
		for(int i =0; i< distances.length ; ++i){
			if(distances[i] < min){
				min=distances[i];
				index = i;
			}
		}
		
		
		return index;
	}
	
	
		
	public static void pickUpBlock(){
		pulleyMotor.setSpeed(300);
		openClaw();
		pulleyDown(5);
		closeClaw();
		pulleyUp(5);
		pulleyDown(1.5);
		openClaw();
		pulleyDown(1.5);
	}
	
	
	public static void openClaw(){
		clawMotor.rotate(CLAW_ANGLE, false);	
	}
	
	
	public static void closeClaw(){
		clawMotor.rotate(-CLAW_ANGLE, false);	
	}
	
	//TODO
	//    the info below is outdated since we improved the crane design, gotta re-take these measurements
	
	//360 = 5 circles = 2.5cm aprox
	//maximum height = 27 circles = 13.5 cm
	//each block = 3cm => maxBlock = 13.5/3 = 4
	
	
	
	public static void pulleyUp(double distance){
		pulleyMotor.rotate(-(int)(FULL_CIRCLE * distance), false);
	}
	
	
	public static void pulleyDown(double distance){
		pulleyMotor.rotate((int)(FULL_CIRCLE * distance), false);
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
		LGZy = lGZy;
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
		LGZx = lGZx;
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
		UGZy = uGZy;
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
		UGZx = uGZx;
	}

}
