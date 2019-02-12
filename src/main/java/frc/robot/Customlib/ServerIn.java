package frc.robot.Customlib;

import org.opencv.core.RotatedRect;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;


public class ServerIn{
    public int port;
    private DatagramSocket socket;
    class AlignmentPacket{
        RotatedRect[] lines;
        RotatedRect centerLine;

    }
    public ServerIn(int port){
        try {
            socket = new DatagramSocket(port, InetAddress.getLocalHost());
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        this.port = port;
    }
    public AlignmentPacket getAlignmentData(){
        byte[] buf = new byte[1024];
        DatagramPacket recivePacket = new DatagramPacket(buf,buf.length);
        socket.receive(recivePacket);
    }

}