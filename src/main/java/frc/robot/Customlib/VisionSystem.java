package frc.robot.Customlib;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import org.opencv.core.RotatedRect;

public class VisionSystem {


	// This class abstracts all of the sensors used of vision and provides member functions
	// to provide access to data prepared from the cameras and lidar

	// Since the vision processing is running on the Raspberry Pi, Network tables should be used to
	// transfer the data from the Pi to the Rio.

    // private String  targetNetTableName ;
    // private String alignmentNetTableName ;
    private Lidar lidar;
	public LimeLight limeLight = new LimeLight();
	public ServerIn server;

    public VisionSystem() {
        this.lidar = new Lidar(I2C.Port.kOnboard);
		this.server = new ServerIn(14580);
    }

	public class BearingData{
		public boolean exactAngle = false;
		public double angle = 0;
		public BearingData(boolean exactAngle, double angle){
			this.exactAngle = exactAngle;
			this.angle = angle;
		}
	}

    public boolean targetIsPresent() {
        return limeLight.hasTarget;
    }

    public void updateVision(){
    	server.updateAlignmentData();

	}
	// The tolerence of bearing to target at which we consider it safe to look at
	// the lidar value.
	// This is to make sure the lidar is pointing at the target and not off into
	// space somewhere.
	static final double LIDAR_BEARING_TOLERANCE = 4.0; // degrees

	// This is the distance estimate at which it is safe to look at the lidar value.
	// If
	// the bot is too far out, the lidar may not be pointing at the target
	// consistently.
	static final double LIDAR_DISTANCE_THRESHOLD = 36.0; // inches

	// This is the tolerence to use when comparing the lidar distance to the visual
	// based estimate.
	// The visual estimate is consistent, but might not be accurate.
	static final double LIDAR_VISUAL_COMPARE_TOLERANCE = 4.0; // inches


	public double distanceToTarget() {
		// This could be as an estimation based on the average height of the bounding
		// rectangles around
		// the reflective tape marking the target. It could be a simple linear
		// relationshi (e.g., each pixel
		// indicates so many inches), or a non-linear relationship. Some testing will
		// help figure it out.
		// I would suggest this value be puseh to the driver station to aid in testing

		// you might also consider working in a check for the alignment line here to to
		// get better
		// accuracy up close. For example, if the alignment line shows we are lines up
		// on target center,
		// the lidar may provide the best estimate of range

		double result = 0.0;
		double lidarDistance = lidar.getDistace(); // wpk to-do need to fetch lidar value
		if (targetIsPresent()) {
			LimeLight.Target3D target = limeLight.getCamTranslation();
			double x = target.translation.x;
			double z = target.translation.z;
			result = Math.sqrt((x*x)+(z*z));
			BearingData b = bearingToTarget();
			if (result < LIDAR_DISTANCE_THRESHOLD && b.exactAngle && Math.abs(b.angle) < LIDAR_BEARING_TOLERANCE) {
				if (Math.abs(result - lidarDistance) < LIDAR_VISUAL_COMPARE_TOLERANCE) {
					result = lidarDistance;
				}
			}
		} else {
			// we can't see the target lines, so look at the alignment line
			if (alignmentLineIsVisible()) {
				BearingData b = bearingToTarget();
				if (Math.abs(b.angle) < LIDAR_BEARING_TOLERANCE) {
					result = lidarDistance;
				}
			}
		}

		return result;
	}


	public BearingData bearingToTarget() {
		// wpk need to put estimation algiorithm here

		// Some thoughts are:
		// If the alignment line is visible and we're sure that it belongs to the target
		// that has been identified (i.e., one end of the line is between, or close, the target hash marks)
		// then a reasonable estimate would be the angle of the RoundedRect bounding the alignment
		// line (might have to flip the sign of the angle). The picture we took earlier and oput in the 
		// presebntation on google show how that works.

		// If the alignment line is not visible or can't be correlated to the target hash lines,
		// then perhaps a bearing estimate can be computed from the difference in heights
		// of the hash marks and the distance between them in pixels as compared to the know 8"
		// distance called out in the game book.


		if(server.getLastPacket() == null)
			return new BearingData(false, 0);
		if(limeLight.getHasTarget()){
			return new BearingData(true,limeLight.getCamTranslation().rotation.y);
		}
			if(server.getLastPacket().Alignmentlines.length == 0){
			//Use the wall rectangle to find bearing

			if(server.getLastPacket().wallRects.length < 2 ) return new BearingData(false,0);

			RotatedRect l = server.getLastPacket().wallRects[0],r = server.getLastPacket().wallRects[0];

			for(RotatedRect rotatedRect : server.getLastPacket().wallRects){
				if(l.center.x > rotatedRect.center.x) l = rotatedRect;
				if(r.center.x < rotatedRect.center.x) r = rotatedRect;
			}

			return new BearingData(false,l.size.height > r.size.height ? -1 : 1); // -1: turn left, 1: turn right
		}

		return new BearingData(false,0);

	}


    public boolean alignmentLineIsVisible() {
		//TODO do something else here
		return getAlignmentRectangle() != null;
	}

	public static final int ALIGNMENT_THRESHOLD = 12; 
	public static int ALIGNMENT_OFFSET = 33;
	public RotatedRect getAlignmentRectangle() {
		ALIGNMENT_OFFSET = (int)SmartDashboard.getNumber("Alignment offset", ALIGNMENT_OFFSET);
		SmartDashboard.putNumber("Alignment offset", ALIGNMENT_OFFSET);
		// wpk this needs to be filled in.
		ArrayList<String> calcs = new ArrayList<String>();
		
		if(server.getLastPacket() == null || server.getLastPacket().Alignmentlines.length == 0)
			return null;
		
		RotatedRect closest = server.getLastPacket().Alignmentlines[0];

		for(RotatedRect rotatedRect : server.getLastPacket().Alignmentlines){
			
			if(Math.abs(rotatedRect.center.x - (limeLight.getXOffset()*320/54-ALIGNMENT_OFFSET)+320/2) <
				Math.abs(closest.center.x - (limeLight.getXOffset()*320/54-ALIGNMENT_OFFSET)+320/2))
			closest = rotatedRect;
		
			// System.out.println("RRC: "+ rotatedRect.center.x + "\t LL XOFF: " + (limeLight.xOffset*320/54));
				
		}
		// SmartDashboard.putStringArray("Calculations",calcs.toArray(String[]::new));
		return closest;
	}
 



}