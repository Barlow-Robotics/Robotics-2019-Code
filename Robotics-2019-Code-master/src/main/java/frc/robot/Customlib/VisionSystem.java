package frc.robot.Customlib;

import org.opencv.core.RotatedRect;

public class VisionSystem {


	// This class abstracts all of the sensors used of vision and provides member functions
	// to provide access to data prepared from the cameras and lidar

	// Since the vision processing is running on the Raspberry Pi, Network tables should be used to
	// transfer the data from the Pi to the Rio.

    private String  targetNetTableName ;
    private String alignmentNetTableName ;
    private Arduino lidar ;


    public void VisionSystem(String c, String alignmentNetTableName,  Arduino lidar ) {
        this.alignmentNetTableName = alignmentNetTableName ;
        this.lidar = lidar ;
    }


    public boolean targetIsPresent() {

        // This function needs to look through the contours returned by the limelight
        // for a pair that tilt toward each other. True if a pair found, false otherwise.

        // Need to consider what to do if more than one pair of lines is found. Do we pick the
        // nearest one? Do we need something to let the operator pick which one?

        return false ;
    }


    public double distanceToTarget() {
		// wpk - need to add code here to read and return the distance to the target as read from the lidar

		// This could be as an estimation based on the average height of the bounding rectangles around
		// the reflective tape marking the target. It could be a simple linear relationshi (e.g., each pixel
		// indicates so many inches), or a non-linear relationship. Some testing will help figure it out.
		// I would suggest this value be puseh to the driver station to aid in testing

		// you might also consider working in a check for the alignment line here to to get better
		// accuracy up close. For example, if the alignment line shows we are lines up on target center,
		// the lidar may provide the best estimate of range
		return 0.0 ;
	}


	public double bearingToTarget() {
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

		return 0.0 ;

	}


    public boolean alignmentLineIsVisible() {
		// wpk - to do
		// return true if the alignment cam sees a line that correlates with the target lines
		return false ; // tbd
	}


	public RotatedRect getAlignmentRectangle() {
		// wpk this needs to be filled in. 
		return new RotatedRect() ;
	}
 



}