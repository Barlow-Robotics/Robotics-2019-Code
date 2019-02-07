package frc.robot.Customlib;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.ArrayList;

public class ServerOut{
    public static int port; 
    private DatagramSocket socket;

    public ServerOut(int port){
        try{
            socket = new DatagramSocket(port);
        }catch(Exception e){System.err.println(e);}
        this.port = port;
    }

    public void sendData(String data){
        byte buf[] = null; 
        buf = data.getBytes(); 
        DatagramPacket packet = new DatagramPacket(buf, buf.length);
        try{
            socket.send(packet);
        }catch(IOException e){System.err.println(e.getCause());}  
    }

}