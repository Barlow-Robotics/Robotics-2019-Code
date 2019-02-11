package frc.robot.Customlib;

//import java.awt.Point;
import java.net.DatagramSocket;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.opencv.core.Point;

public class piPacketHandler{
    
    public Point center;
    public Size size;
    public double angle;
   
    public RotatedRect rec;

    public piPacketHandler(Point center, Size size, double angle){
        this.center = center;
        this.size = size;
        this.angle = angle;

        this.rec = new RotatedRect(center, size, angle);
    }

    public double getCorrectedRectAngle(){
        if(angle > 90){
            return angle - 180;
        }else{
            return angle;
        }
    }
}
