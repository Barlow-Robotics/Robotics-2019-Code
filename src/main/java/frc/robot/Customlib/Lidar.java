package frc.robot.Customlib;

import edu.wpi.first.wpilibj.I2C;

public class Lidar {

    private DistanceUpdater distUpdater;
    private Thread lidarThread;

    public Lidar(I2C.Port port) {
        distUpdater = new DistanceUpdater(port, this);
        lidarThread = new Thread(distUpdater);
        lidarThread.start();
    }

    public double getDistance() {
        return accessDistance(true, 0);
    }

    private int distance = 0;

    public synchronized int accessDistance(boolean read, int nDist) {
        if (read)
            return distance;
        else
            distance = nDist;

        return 0;
    }


}
