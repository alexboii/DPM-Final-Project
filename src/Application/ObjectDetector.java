package Application;
import SensorData.LSPoller;
import SensorData.USPoller;
import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

public class ObjectDetector extends Thread {

	private final double COLOR_SENSOR_THERSHOLD = 10; 
	private final double DISTANCE_THRESHOLD = 20; 
	private TextLCD t;
	private LSPoller lsPoller;
	private USPoller usPoller, usPoller2;

	// constructor
	public ObjectDetector(USPoller usPoller, USPoller usPoller2, TextLCD t) {
		this.usPoller = usPoller;
		this.usPoller2 = usPoller2;
		this.t = t;
	}

	//
	public void run() {
		while (true) {

			t.clear();

			while (usPoller.getDistance() < DISTANCE_THRESHOLD && usPoller2.getDistance() < DISTANCE_THRESHOLD) {
				t.drawString("Blue block", 0, 0);

			}
			// NO OBJECT IN SIGHT
			t.drawString("No Object", 0, 0);
		}
	}
}
