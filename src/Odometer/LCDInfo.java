package Odometer;
import SensorData.USPoller;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

/**
 * Continuously display odometer's information on the EV3's screen
 * @author Unknown, a TA, probably 
 *
 */
public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 800;
	private Odometer odo;
	private Timer lcdTimer;
	private USPoller highUs, lowUs;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;
	
	// arrays for displaying data
	private double [] pos;
	
	/**
	 * Constructor
	 * @param odo Odometer
	 */
	public LCDInfo(Odometer odo, USPoller highUs, USPoller lowUs) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		this.highUs = highUs;
		this.lowUs = lowUs;

		
		// initialise the arrays for displaying data
		pos = new double [3];
		
		// start the timer
		lcdTimer.start();
	}
	
	/**
	  * {@inheritDoc}
	  */
	public void timedOut() { 
		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawInt((int)(pos[0]), 3, 0);
		LCD.drawInt((int)(pos[1]), 3, 1);
		LCD.drawInt((int)Math.toDegrees(pos[2]), 3, 2);
		
		LCD.drawString("UP: ", 0, 4);
		LCD.drawString("DN: ", 0, 5);
		LCD.drawInt((int)(highUs.getDistance()), 4, 4);
		LCD.drawInt((int)(lowUs.getDistance()), 4, 5);
		
		
		
	}
}
