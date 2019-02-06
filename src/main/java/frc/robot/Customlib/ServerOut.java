package frc.robot.Customlib;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.util.ArrayList;

public class ServerOut{
    public static int port; 
    private DatagramSocket socket;

    public ServerOut(int port){
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