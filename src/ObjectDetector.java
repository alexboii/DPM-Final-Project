import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

public class ObjectDetector extends Thread {

	private final double COLOR_SENSOR_THERSHOLD = 3.5; // CM
	private final double DISTANCE_THRESHOLD = 100.0; // cm
	private TextLCD t;
	private LSPoller lsPoller;
	private USPoller usPoller;

	// constructor
	public ObjectDetector(USPoller usPoller, LSPoller lsPoller, TextLCD t) {
		this.usPoller = usPoller;
		this.lsPoller = lsPoller;
		this.t = t;
	}

	//
	public void run() {
		while (true) {

			t.clear();
			if (usPoller.getDistance() < DISTANCE_THRESHOLD) {
				if (lsPoller.isBlue()) {
					t.clear();
					System.out.println("Styrofoam object detected");
				} else {
					t.clear();
					System.out.println("Non-styrofoam object detected");
//					t.drawString("Non-styrofoam object detected");
				}
			} else {
				t.drawString("No object detected", 0, 0);
			}
		}
	}
}
