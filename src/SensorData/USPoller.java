package SensorData;
import lejos.robotics.SampleProvider;

public class USPoller extends Thread {

	private double distance;

	private SampleProvider us;
	private float[] usData;
	private Object lock;

	public USPoller(SampleProvider us, float[] usData) {
		this.us = us;
		this.usData = usData;
		this.lock = new Object();
	}

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

	public double getDistance() {
		synchronized (lock) {
			return distance;
		}
	}
}