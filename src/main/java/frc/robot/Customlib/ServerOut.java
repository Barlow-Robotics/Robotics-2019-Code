package frc.robot.Customlib;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.ArrayList;

public class ServerOut{
    public int port;
    public static String ip; 
    private DatagramSocket socket;
    public ServerOut(int port){
        this(port,"10.45.72.41");
    }
    public ServerOut(int port,String ip){
    	try {
			socket = new DatagramSocket();
		} catch (SocketException e) {
			e.printStackTrace();
        }
        this.ip = ip;
        this.port = port;
    }

    public void sendData(String data){
        byte buf[] = null; 
        buf = data.getBytes(); 
        DatagramPacket packet;
		try {
			packet = new DatagramPacket(buf, buf.length,InetAddress.getByName(ip),port);
			socket.send(packet);
		} catch (Exception e1) {

		}
    }

}