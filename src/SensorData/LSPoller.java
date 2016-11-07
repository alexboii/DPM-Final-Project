package SensorData;
import lejos.robotics.SampleProvider;

/**
 * This class polls data from the light sensor.
 * @author Alex
 *
 */
public class LSPoller extends Thread {
	// variables
	private boolean blue = false;
	private Object lock;
	private SampleProvider colorSensor;
	private float[] Val;
	private int blueCounter;

	
	/**
	 * Constructor
	 * @param colorSensor Colour Sensor
	 * @param Val RBG values
	 */
	public LSPoller(SampleProvider colorSensor, float[] Val) {
		this.colorSensor = colorSensor;
		this.Val = Val;
		this.lock = new Object();
	}

	/**
	  * {@inheritDoc}
	  */
	public void run() {
		blueCounter = 0;

		while (true) {
			synchronized (lock) {

				colorSensor.fetchSample(Val, 0);

				// IF OBJECT IS BLUE, THEN SET VARIABLE TO TRUE
				if (Val[0] < Val[1]) {
					blueCounter++;
					if (blueCounter > 5) {
						blue = true;
						blueCounter = 0;
					}
				} else {
					blue = false;
				}
				try {
					Thread.sleep(20);
				} catch (Exception e) {
				} 
			}
		}
	}

	/**
	 * Return if block is blue 
	 * @return Block is blue
	 */
	public boolean isBlue() {
		synchronized (lock) {
			return this.blue;
		}
	}
}