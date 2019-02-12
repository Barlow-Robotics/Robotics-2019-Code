package frc.robot.Customlib;

import org.opencv.core.RotatedRect;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import com.google.gson;

public class ServerIn{
    public int port;
    private DatagramSocket socket;
    private Gson gson = new Gson();
    private AlignmentPacket lastPkt;

    class RotatedRectArray{ public RotatedRect[] lines;}
    class AlignmentPacket{
        public RotatedRect[] Alignmentlines;
        public RotatedRect[] wallRects;
        public RotatedRect centerLine;
        public AlignmentPacket(RotatedRect[] Alignmentlines, RotatedRect[] wallRects, RotatedRect centerLine){
            this.Alignmentlines = Alignmentlines;
            this.wallRects = wallRects;
            this.centerLine = centerLine;
        }
    }


    public ServerIn(int port){
        try {
            socket = new DatagramSocket(port, InetAddress.getLocalHost());
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        this.port = port;
        updateAlignmentData();
    }
    public AlignmentPacket getLastPacket(){
        return lastPkt;
    }
    public void updateAlignmentData(){
        byte[] buf = new byte[1024];
        DatagramPacket recivePacket = new DatagramPacket(buf,buf.length);
        try {
            socket.receive(recivePacket);
        }catch(Exception e){return;}
        String data = new String(recivePacket.getData(), StandardCharsets.UTF_8);
        lastPkt = string2Packet(data);
    }
    public AlignmentPacket string2Packet(String pkt){
        return gson.fromJson(pkt,RotatedRectArray.class);
    }
}