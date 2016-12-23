package Wifi;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.HashMap;

/**
 * * This class opens a wifi connection, waits for the data and then allows
 * access to the data after closing the wifi socket.
 * 
 * It should be used by calling the constructor which will automatically wait
 * for data without any further user command.
 * 
 * Then, once completed, it will allow access to an instance of the Transmission
 * class which has access to all of the data needed.
 * 
 * @author Sean Lawlor
 * 
 */

public class WifiConnection {

	public HashMap<String, Integer> StartData;

	/**
	 * Constructor
	 * 
	 * @param serverIP
	 *            EV3 block's IP
	 * @param teamNumber
	 *            Team 14
	 * @throws IOException
	 */
	public WifiConnection(String serverIP, int teamNumber) throws IOException {
		this(serverIP, teamNumber, true);
	}

	/**
	 * Constructor
	 * 
	 * @param serverIP
	 *            EV3 block's IP
	 * @param teamNumber
	 *            Team 14
	 * @param debugPrint
	 *            Print on Eclipse's Terminal
	 * @throws IOException
	 */
	public WifiConnection(String serverIP, int teamNumber, boolean debugPrint) throws IOException {

		// Open connection to the server and data streams
		int port = 2000 + teamNumber; // semi-abritrary port number"
		Socket socketClient = new Socket(serverIP, port);

		DataOutputStream dos = new DataOutputStream(socketClient.getOutputStream());
		DataInputStream dis = new DataInputStream(socketClient.getInputStream());

		if (debugPrint) {
			System.out.println("Connected\nWaiting for data");
		}

		// Wait for the server transmission to arrive
		while (dis.available() <= 0)
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
			}

		// Parse transmission
		this.StartData = ParseTransmission.parseData(dis);

		if (debugPrint) {
			System.out.println("Data received");
		}

		// End the wifi connection
		dis.close();
		dos.close();
		socketClient.close();

	}

	/**
	 * @return Data Received from Client
	 */
	public HashMap<String, Integer> getStartData() {
		return StartData;
	}

}
