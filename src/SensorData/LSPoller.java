package SensorData;

import lejos.robotics.SampleProvider;

/**
 * This class polls data from the light sensor.
 * 
 * @author Alexander Bratyshkin
 *
 */
public class LSPoller extends Thread {
	// variables
	private Object lock;
	private SampleProvider colorSensor;
	private float[] Val;
	private double intensity;

	/**
	 * Constructor
	 * 
	 * @param colorSensor
	 *            Colour Sensor
	 * @param Val
	 *            RBG values
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

		while (true) {
			synchronized (lock) {
				colorSensor.fetchSample(Val, 0);
				waitMs(20);
			}
		}
	}

	public void waitMs(long time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// Do nothing, it should never happen
			e.printStackTrace();
		}
	}

	/**
	 * @return the intensity of the light caught my the ultrasonic sensor
	 */
	public int getLightLevel() {
		return (int) (100 * this.Val[0]);
	}

}
