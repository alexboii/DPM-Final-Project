package SensorData;
import Navigation.Navigation;
import Odometer.OdometerCorrection;
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
	private double intensity;
	private final double TRESHOLD = 30;
	private OdometerCorrection odoCor;
	
	/**
	 * Constructor
	 * @param colorSensor Colour Sensor
	 * @param Val RBG values
	 */
	public LSPoller(SampleProvider colorSensor, float[] Val, OdometerCorrection odoCor) {
		this.colorSensor = colorSensor;
		this.Val = Val;
		this.lock = new Object();
		this.odoCor = odoCor;
	}

	/**
	  * {@inheritDoc}
	  */
	public void run() {

		while (true) {
			synchronized (lock) {

				colorSensor.fetchSample(Val, 0);

				intensity = Val[0] * 100;
				
				
				if(intensity < TRESHOLD){
					
				}
				
				
				// IF OBJECT IS BLUE, THEN SET VARIABLE TO TRUE

				
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
	
	
	
public void crossedLine(){
	odoCor.correct();
}
	
	
	
}
