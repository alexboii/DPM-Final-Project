import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

//LAB 5 OBJECT SEARCHING
//WEIPING REN - 260613810

public class Lab5 {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor usMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor colorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort = LocalEV3.get().getPort("S2");
	public static final double TRACK = 15.1;
	public static final double WHEEL_RADIUS = 2.1;
	
	public static final int[][] WAYPOINTS_DEMO_2 = { { -60, 60 } };


	public static void main(String[] args) {

		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");
		float[] usData = new float[usValue.sampleSize()];

		final TextLCD t = LocalEV3.get().getTextLCD();

		USLocalizer.LocalizationType type = USLocalizer.LocalizationType.FALLING_EDGE;

		Odometer odometer = new Odometer(leftMotor, rightMotor, 50, true);

		USPoller usPoller = new USPoller(usValue, usData);
//		ObjectDetector objectDetect = new ObjectDetector(usPoller, lsPoller, t);

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
//			lsPoller.start();
			usPoller.start();
//			objectDetect.start();
		} else {

			t.clear();
			LCDInfo lcd = new LCDInfo(odometer);
			Navigation navigator = new Navigation(odometer);

			USLocalizer usl = new USLocalizer(odometer, usValue, usData, type);
			usl.doLocalization(navigator);

			// while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			// ;

			EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
			SampleProvider colorValueLoc = colorSensor.getMode("Red");
			float[] colorDataLoc = new float[colorValueLoc.sampleSize()];

//			colorMotor.rotate(-90);
			LightLocalizer lsl = new LightLocalizer(odometer, colorValueLoc, colorDataLoc);
			lsl.doLocalization(navigator);

			while (Button.waitForAnyPress() != Button.ID_ESCAPE)
				;
			colorMotor.rotate(90);
			colorSensor.setFloodlight(lejos.robotics.Color.WHITE);
			SampleProvider colorValue = colorSensor.getMode("RGB"); 
			float[] colorData = new float[colorValue.sampleSize()];
			LSPoller lsPoller = new LSPoller(colorValue, colorData);

			lsPoller.start();
			usPoller.start();
			OdometerAvoidance odometerAvoid = new OdometerAvoidance(leftMotor, rightMotor, TRACK);
			odometerAvoid.start();
			OdometryDisplay odometryDisplay = new OdometryDisplay(odometerAvoid, t);
			odometryDisplay.start();
			NavigatorObstacle navigatorObstacle = new NavigatorObstacle(odometerAvoid, leftMotor, rightMotor, WAYPOINTS_DEMO_2, usMotor, usPoller, true, TRACK);
			new Thread(navigatorObstacle).start();
			
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);

	}

}
