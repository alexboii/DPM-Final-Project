package SensorData;
import lejos.robotics.SampleProvider;

/**
 * This class polls data from the ultrasonic sensors independently of each other.
 * @author Alex
 *
 */
public class USPoller extends Thread {

	private double distance;

	private SampleProvider us;
	private float[] usData;
	private Object lock;

	/**
	 * Constructor
	 * @param us Ultrasonic Sensor's Sample Provider
	 * @param usData Distance Data 
	 */
	public USPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
		this.lock = new Object();
	}

	/**
	  * {@inheritDoc}
	  */
	public void run() {

		while (true) {
			synchronized (lock) {
				us.fetchSample(usData, 0);// acquire data
				if (usData[0] >= 255.0) {
					continue;
				}
				distance = (usData[0] * 100.0);
				try {
					Thread.sleep(20);
				} catch (Exception e) {
				} // timed sampling rate
			}
		}
	} 

	/**
	 * @return Distance Read by Ultrasonic Sensor
	 */
	public double getDistance() {
		synchronized (lock) {
			return distance;
		}
	}
}