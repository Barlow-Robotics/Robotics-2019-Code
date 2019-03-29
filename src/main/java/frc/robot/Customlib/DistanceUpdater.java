package frc.robot.Customlib;

import edu.wpi.first.wpilibj.I2C;


class DistanceUpdater implements Runnable {

    private final int LIDARLITE_ADDR_DEFAULT = 0x62;
    private final int READ_WRITE_DELAY = 40 ;

    Lidar lidar ;

    private I2C i2c;

    public DistanceUpdater(I2C.Port port, Lidar l) {
        lidar = l ;
        i2c = new I2C(port, LIDARLITE_ADDR_DEFAULT);
        i2c.write(0x02, 0x1d);
    }

    @Override
    public void run() {
        while (true) {
            lidar.accessDistance(false, getDistance());
        }
    }

    public int getDistance() {
        byte[] buffer;
        buffer = new byte[2];
        i2c.write(0x00, 0x04);

        try {
            Thread.sleep(READ_WRITE_DELAY);
        } catch (Exception e) {
        }
        i2c.read(0x8f, 2, buffer);

        int distance = (int)(((double)Integer.toUnsignedLong(buffer[0] << 8) + Byte.toUnsignedInt(buffer[1]))/2.5) ;
        //return (int) Integer.toUnsignedLong(buffer[0] << 8) + Byte.toUnsignedInt(buffer[1]);
        return distance ;
    }
}
