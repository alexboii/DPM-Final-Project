
package Wifi;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.HashMap;

/**
 * 
 * Static parsers for parsing data off the communication channel
 * 
 * The order of data is defined in the Server's Transmission class
 * 
 * @author Sean Lawlor, Modified by P. Ferrie 
 *
 */
public class ParseTransmission {

	// This should only be called after verifying that there is data in the
	// input stream
	/**
	 * Parses the data received from the Client
	 * 
	 * @param dis
	 *            Data from Client
	 * @return Data from Client
	 */
	@SuppressWarnings("unchecked")
	public static HashMap<String, Integer> parseData(DataInputStream dis) {
		HashMap<String, Integer> StartData;
		try {
			ObjectInputStream ois = new ObjectInputStream(dis);
			StartData = (HashMap<String, Integer>) ois.readObject();
		} catch (Exception e) {
			StartData = null;
		}
		return StartData;
	}

	/**
	 * Ignores DataInputStream
	 * 
	 * @param dis
	 *            Data from Client
	 * @throws IOException
	 */
	public static void ignore(DataInputStream dis) throws IOException {
		dis.readChar();
	}

}
