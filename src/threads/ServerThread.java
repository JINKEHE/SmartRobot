package threads;
import java.io.*;
import java.net.*;

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
		}
	}
	
	public void run() {
		try {
			ServerSocket server = new ServerSocket(port);
			Socket client = server.accept();
			OutputStream out = client.getOutputStream();
			dOut = new DataOutputStream(out);
			ableToSend = true;
			while (client.isConnected() && !client.isClosed()) {
				
			}
			ableToSend = false;
			server.close();
		} catch (Exception e) {
		}
	}
}
