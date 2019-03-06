package frc.robot.Customlib;

import org.opencv.core.RotatedRect;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketAddress;
import java.net.SocketTimeoutException;
import java.nio.charset.StandardCharsets;
import com.google.gson.*;
import com.google.gson.stream.JsonReader;
import java.io.* ;

public class ServerIn{
    private DatagramSocket socket;
    private Gson gson = new Gson();
    private AlignmentPacket lastPkt;

    public class AlignmentPacket{
        public RotatedRect[] Alignmentlines;
        public RotatedRect[] wallRects;
        public double LLBearing,LLRange,lidarDist;
        public double xOffset;

        public AlignmentPacket(RotatedRect[] Alignmentlines, RotatedRect[] wallRects,double LLBearing, double LLRange, double lidarDist, double xOffset){
            this.Alignmentlines = Alignmentlines;
            this.wallRects = wallRects;
            this.LLBearing = LLBearing;
            this.LLRange = LLRange;
            this.lidarDist = lidarDist;
        }
    }


    public ServerIn(int port){
        try {
            socket = new DatagramSocket(port);
            socket.setSoTimeout(1);
        } catch (Exception e) {
            e.printStackTrace();
        }
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
        }catch(SocketTimeoutException e){
            //System.out.println("No packet!");
            return;
        }catch(Exception e){
            System.err.println(e);
            return;
        }
        
        String data = new String(recivePacket.getData(), StandardCharsets.UTF_8);
        //System.out.println("Data: " + data);
        lastPkt = string2Packet(data);
    }
    public AlignmentPacket string2Packet(String pkt){
        JsonReader reader = new JsonReader( new StringReader(pkt)) ;
        reader.setLenient(true ) ;
        try{
            return gson.fromJson(reader,AlignmentPacket.class);
        }catch(Exception e){
            return lastPkt;
        }
    }
}