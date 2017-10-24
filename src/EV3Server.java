import java.io.*;
import java.net.*;
import lejos.hardware.Button;
import lejos.hardware.Keys;

public class EV3Server extends Thread {

	public static final int port = 12345;
	private SimpleRobot myRobot;
	private int delay;
	private String str;
	private boolean need = false;
	
	public EV3Server(SimpleRobot myRobot, int delay) {
		this.myRobot = myRobot;
		this.delay = delay;
		setDaemon(true);
	}
	
	public void send(String str) {
		System.out.println(str);
		need = true;
		this.str = str;
	}
	
	public void run() {
		try {
			ServerSocket server = new ServerSocket(port);
			Socket client = server.accept();
			OutputStream out = client.getOutputStream();
			DataOutputStream dOut = new DataOutputStream(out);
			while (client.isConnected() && !client.isClosed()) {
				/*
				if (Button.getButtons() != Keys.ID_ESCAPE) {
					myRobot.stop();
				}
				*/
				if (need = true){
					dOut.writeUTF(this.str);
					dOut.flush();
					need = false;
				}
				try{
		    		sleep(delay);
		    	}
		    	catch(Exception e){
		    	}
	
			}
			System.out.println("88888");
			server.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
