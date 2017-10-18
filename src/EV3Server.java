import java.io.*;
import java.net.*;
import lejos.hardware.Battery;

/**
 * Maximum LEGO EV3: Building Robots with Java Brains
 * ISBN-13: 9780986832291
 * Variant Press (C) 2014
 * Chapter 14 - Client-Server Robotics
 * Robot: EV3 Brick
 * Platform: LEGO EV3
 * @author Brian Bagnall
 * @version July 20, 2014
 */
public class EV3Server {

	public static final int port = 1234;
	
	public static void main(String[] args) throws IOException {
		ServerSocket server = new ServerSocket(port);
		System.out.println("Awaiting client..");
		Socket client = server.accept();
		System.out.println("CONNECTED");
		OutputStream out = client.getOutputStream();
		DataOutputStream dOut = new DataOutputStream(out);
		dOut.writeUTF("Battery: " + Battery.getVoltage());
		dOut.flush();
		server.close();
	}
}
