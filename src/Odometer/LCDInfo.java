package Odometer;
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
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();
	private static String label1, label2, label3;
	private  static int value1, value2, value3;

	
	// arrays for displaying data
	private double [] pos;
	
	/**
	 * Constructor
	 * @param odo Odometer
	 */
	public LCDInfo(Odometer odo) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
	
		
		label1 = " ";
		label2 = " ";
		label3 = " ";
		
		value1 = 0;
		value2 = 0;
		value3 = 0;
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
		LCD.drawString("T: ", 0, 2);
		
		
		
		
		LCD.drawInt((int)(pos[0]), 3, 0);
		LCD.drawInt((int)(pos[1]), 3, 1);
		LCD.drawInt((int)(pos[2]), 3, 2);
		
		
		LCD.drawString(label1, 0, 3);
		LCD.drawString(label2, 0, 4);
		LCD.drawString(label3, 0, 5);

	
		LCD.drawInt((value1 ), 4, 3);
		LCD.drawInt(value2, 4, 4);
		LCD.drawInt(value3, 4, 5);
		
		
		
	}

	public String getLabel1() {
		return label1;
	}

	public static void setLabel1(String nlabel1) {
		label1 = nlabel1;
	}

	public String getLabel2() {
		return label2;
	}

	public static void setLabel2(String nlabel2) {
		label2 = nlabel2;
	}

	public String getLabel3() {
		return label3;
	}

	public static void setLabel3(String nlabel3) {
		label3 = nlabel3;
	}

	public int getValue1() {
		return value1;
	}

	public static void setValue1(int nvalue1) {
		value1 = nvalue1;
	}

	public int getValue2() {
		return value2;
	}

	public static void setValue2(int nvalue2) {
		value2 = nvalue2;
	}

	public int getValue3() {
		return value3;
	}

	public static void setValue3(int nvalue3) {
		value3 = nvalue3;
	}
}
