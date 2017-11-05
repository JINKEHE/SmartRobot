package threads;
import java.io.*;
import java.net.*;

// this thread makes the robot a server and sends important data to the client
// for example, the distances detected by the sensors
// the robot will create and run such a thread when it starts
// initially, this thread is used to debug
public class ServerThread extends Thread {

	public static final int port = 12345;
	private DataOutputStream dOut;
	private boolean ableToSend = false;
	
	public ServerThread() {
		setDaemon(true);
	}
	
	public void sendToClient(String str){
		try {
			if (ableToSend) {
				dOut.writeUTF(str);
				dOut.flush();
			}
		} catch (IOException e) {
			System.out.println("Failed to send a message.");
		}
	}
	
	public void run() {
		try {
			// setup the socket and the output stream
			ServerSocket server = new ServerSocket(port);
			Socket client = server.accept();
			OutputStream out = client.getOutputStream();
			dOut = new DataOutputStream(out);
			// the server is able to send messages after the stream is built up
			ableToSend = true;
			while (client.isConnected() && !client.isClosed()) {
				// running normally
				// waiting for a message to send
			}
			// the server is going to close
			ableToSend = false;
			server.close();
		} catch (Exception e) {
			System.out.println("Disconnected.");
		}
	}
}
