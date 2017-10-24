import java.io.*;
import java.net.Socket;

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
public class PCClient {
	public static void main(String[] args) throws IOException {
		String ip = "10.0.1.1"; // BT
		if(args.length > 0) ip = args[0];
		while (true) {
			try{
				boolean connected = false;
				Socket sock = null;
				System.out.println("=================================================");
				while (!connected) {
					try{
						sock = new Socket(ip, EV3Server.port);
						connected = true;
					} catch (Exception e) {
						
					}
				}
				System.out.println("Connected");
				InputStream in = sock.getInputStream();
				DataInputStream dIn = new DataInputStream(in);
				while (sock.isConnected() && !sock.isClosed()) {
					String str = dIn.readUTF();
					System.out.println(str);
				}
				sock.close();
			} catch(Exception e) {
				
			}
		}
	}
}
