package main;
import threads.ServerThread;
import java.io.*;
import java.net.Socket;

public class PCClient {
	
	private static final String SEPARATOR= "======================================================";
	
	public static void main(String[] args) throws IOException {
		String ip = "10.0.1.1";
		Socket sock = null;
		while (true) {
			try{
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
				while (!finished) {
					message = dIn.readUTF();
					System.out.println(message);
				}
				sock.close();
			} catch(Exception e) {
			} finally {
			}
		}
	}
}
