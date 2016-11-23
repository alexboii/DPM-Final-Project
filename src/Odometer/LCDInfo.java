package Odometer;
import SensorData.LSPoller;
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
	private TextLCD LCD = LocalEV3.get().getTextLCD();
	private LSPoller lsPoller;

	
	// arrays for displaying data
	private double [] pos;
	
	/**
	 * Constructor
	 * @param odo Odometer
	 */
	public LCDInfo(Odometer odo, USPoller highUs, USPoller lowUs, LSPoller lsPoller) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		this.highUs = highUs;
		this.lowUs = lowUs;
		this.lsPoller = lsPoller;
		
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
		
		LCD.drawString("LS: ", 0, 3);
		LCD.drawInt((lsPoller.getLightLevel() ), 4, 3);

		
		LCD.drawString("UP: ", 0, 4);
		LCD.drawString("DN: ", 0, 5);
		LCD.drawInt((int)(highUs.getFilteredDistance()), 4, 4);
		LCD.drawInt((int)(lowUs.getFilteredDistance()), 4, 5);
		
		
		
	}
}
