package main;
import threads.ServerThread;
import java.io.*;
import java.net.Socket;

// the client can receive and print the messages sent by the server
// the server and client work like a monitor of the robot
// the client should be run by the PC rather than the robot
// initially, the server and client were designed for debug
public class PCClient {
	
	private static final String SEPARATOR= "======================================================";
	
	public static void main(String[] args) throws IOException {
		// the IP address of the remote robot
		String ip = "10.0.1.1";
		Socket sock = null;
		while (true) {
			try{
				// automatically connect the robot when it's online
				boolean connected = false;
				while (!connected) {
					sock = new Socket(ip, ServerThread.port);
					connected = true;
					System.out.println(SEPARATOR);
				}
				InputStream in = sock.getInputStream();
				DataInputStream dIn = new DataInputStream(in);
				System.out.println("Connected");
				String message = null;
				boolean finished = false;
				// print whatever messages the robot receives
				while (!finished) {
					message = dIn.readUTF();
					System.out.println(message);
				}
				sock.close();
			} catch(Exception e) {
				System.out.println("Disconnected.");
			} 
		}
	}
}
