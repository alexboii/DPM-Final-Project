package SensorData;

import java.util.Arrays;
import java.util.LinkedList;

import lejos.robotics.SampleProvider;

/**
 * This class polls data from the ultrasonic sensors independently of each
 * other.
 * 
 * @author Alex
 *
 */
public class USPoller extends Thread {

	private double distance;

	private SampleProvider us;
	private float[] usData;
	private Object lock;
	private static final int MEDIAN_FILTER_WINDOW = 5;
	private static LinkedList<Float> medianFilter = new LinkedList<Float>();

	/**
	 * Constructor
	 * 
	 * @param us
	 *            Ultrasonic Sensor's Sample Provider
	 * @param usData
	 *            Distance Data
	 */
	public USPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
		this.lock = new Object();

		for (int i = 0; i < MEDIAN_FILTER_WINDOW; i++) {
			medianFilter.add(0f);
		}

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

		return distance;

	}

	public float getFilteredDistance() {

		// this.us.fetchSample(usData, 0);
		// float distance = usData[0] * 100;
		float returnDistance = 0;


		if (distance > 80) {
			distance = 255;
		}

		float median = getMedian(medianFilter);
		if (distance < median) {
			returnDistance = median;
		} else {
			returnDistance = (float) distance;
		}

		medianFilter.poll();
		medianFilter.add((float) distance);

		return returnDistance;

	}

	private float getMedian(LinkedList<Float> f) {
		Float[] array = f.toArray(new Float[f.size()]);
		Arrays.sort(array);

		float median = 0f;
		if (array.length % 2 == 0) {
			median = 0.5f * (array[array.length / 2] + array[array.length / 2 - 1]);
		} else {
			median = array[(int) Math.floor((float) array.length / 2f)];
		}
		return median;

	}
}