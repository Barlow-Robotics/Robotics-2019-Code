package frc.robot.Customlib;

import org.opencv.core.RotatedRect;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import com.google.gson.*;

public class ServerIn{
    public int port;
    private DatagramSocket socket;
    private Gson gson = new Gson();
    private AlignmentPacket lastPkt;

    class AlignmentPacket{
        public RotatedRect[] Alignmentlines;
        public RotatedRect[] wallRects;
        public RotatedRect centerLine;
        public double LLBearing,LLRange,lidarDist;

        public AlignmentPacket(RotatedRect[] Alignmentlines, RotatedRect[] wallRects, RotatedRect centerLine,double LLBearing, double LLRange, double lidarDist){
            this.Alignmentlines = Alignmentlines;
            this.wallRects = wallRects;
            this.centerLine = centerLine;
            this.LLBearing = LLBearing;
            this.LLRange = LLRange;
            this.lidarDist = lidarDist;
        }
    }


    public ServerIn(int port){
        try {
            socket = new DatagramSocket(port, InetAddress.getLocalHost());
        } catch (Exception e) {
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
        return gson.fromJson(pkt,AlignmentPacket.class);
    }
}