package frc.robot.Customlib;

import edu.wpi.first.wpilibj.I2C;


public class Lidar{
    public final int LIDARLITE_ADDR_DEFAULT = 0x62;

    private DistanceUpdater distUpdater;
    private Thread lidarThread;
    public Lidar(I2C.Port port) {
        distUpdater = new DistanceUpdater(port);
        lidarThread = new Thread(distUpdater);
        lidarThread.start();
    }
    public double getDistace(){
        return accessDistance(true,0);
    }

    private static int distance = 0;

    public static synchronized int accessDistance(boolean read, int nDist){
        if(read) return distance;
        else distance = nDist;
        return 0;
    }
    class DistanceUpdater implements Runnable{
        private I2C i2c;
        public DistanceUpdater(I2C.Port port){
            i2c = new I2C(port,LIDARLITE_ADDR_DEFAULT);
            i2c.write(0x02,0x1d);
        }
        @Override
        public void run(){
            while(true){
                Lidar.accessDistance(false,getDistance());
            }
        }
        public int getDistance() {
            byte[] buffer;  	
            buffer = new byte[2];
            i2c.write(0x00, 0x04);

            try{
                Thread.sleep(40);
            }catch(Exception e){}
            i2c.read(0x8f, 2, buffer);  	
    
    
            return (int)Integer.toUnsignedLong(buffer[0] << 8) + Byte.toUnsignedInt(buffer[1]);	
        }
    }

}