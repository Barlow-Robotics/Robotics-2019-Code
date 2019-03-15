package frc.robot.Customlib;

import java.util.*;
import org.opencv.core.RotatedRect;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Limelight library for easy use of the Limelight camera. Use it as you wish.
 */
public class LimeLight  {		
	//Create variables
	public double targetD;
	public boolean hasTarget;
	public double xOffset;
	public double yOffset;
	public double area;
	public double skew;
	public double LEDMode;
	public double camMode;
	public double pipeline;
	public double horiz;
	public double vert;
	public double X_OFFSET = -.85;
ArrayList<Target3D> longHistory = new ArrayList<Target3D>();
ArrayList<Target3D> adjustedHistory = new ArrayList<Target3D>();

public static final int averageLength = 3;
public static final int longHistoryLength = 10;
public static final int xTolerance = 10;

public LimeLight(){
	for(int i = 0; i < averageLength; i++)
		adjustedHistory.add(new Target3D(0,0,0,0,0,0));

	for(int i = 0; i < longHistoryLength; i++)
		longHistory.add(new Target3D(0,0,0,0,0,0));

}


	//Limelight specs will be stored in this class
	class specs{
		//resolution of camera
		class resolution{
			public static final int height = 240;
			public static final int width = 320;
		}
		class fov{
			public static final int vertical = 41;
			public static final int horizontal = 54;
		}

	}
	public class Vector3f{
		public double x,y,z;
		public Vector3f(double x, double y, double z){
			this.x = x;
			this.y = y;
			this.z = z;
		}
		public void add(Vector3f other){
			x += other.x;
			y += other.y;
			z += other.z;
		}
		public void multiplyScalar(double Scalar){
			x *= Scalar;
			y *= Scalar;
			z *= Scalar;
		}
	}
	public class Target3D{

		public Vector3f translation, rotation;
		public Target3D(Vector3f translation, Vector3f rotation){
			this.translation = translation;
			this.rotation = rotation;
			
		}
		public Target3D(double x1, double y1, double z1, double x2, double y2, double z2){
			this(new Vector3f(x1,y1,z1),new Vector3f(x2, y2, z2));
		}
		
	}
	
	/**
	 * Get the network table for the limelight
	 * @return Returns the network table for the limelight
	 */
	public NetworkTable getLimetable() {
		return NetworkTableInstance.getDefault().getTable("limelight");
	}

	
	/**
	 * Does the camera proccessor have a target?
	 * @return Returns true if the limelight has a target, false if it doesn't
	 */
	public boolean getHasTarget() {
		targetD = getLimetable().getEntry("tv").getDouble(0); 
		if(targetD == 0) {
			hasTarget = false;
		}else if(targetD == 1) {
			hasTarget = true;
		}
		return hasTarget;
	}
	
	/**
	 * Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
	 * @return Returns the X offset from the crosshair
	 */
	public double getXOffset() {
		xOffset = getLimetable().getEntry("tx").getDouble(1);
		return xOffset;
	}
	
	/**
	 * Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
	 * @return Returns the Y offset from the crosshair
	 */
	public double getYOffset() {
		yOffset = getLimetable().getEntry("ty").getDouble(0);
		return yOffset;
	}
	
	/**
	 * Target Area (0% of image to 100% of image)
	 * @return Returns the % of area that the target area takes up
	 */
	public double getArea() {
		area = getLimetable().getEntry("ta").getDouble(0);
		return area;
	}
	
	/**
	 * Skew or rotation (-90 degrees to 0 degrees)
	 * @return gets the skew or rotation of the target area (-90 degrees to 0 degrees)
	 */
	public double getSkew() {
		skew = getLimetable().getEntry("ts").getDouble(0);
		return skew;
	}
	
	/**
	 * Limelight LED state
	 * @return Returns the current LED mode of the Limelight
	 */
	public double getLEDMode() {
		LEDMode = getLimetable().getEntry("ledMode").getDouble(1);
		return LEDMode;
	}

	public double getHorizontalLength(){
		horiz = getLimetable().getEntry("thoriz").getDouble(0);
		return horiz;
	}
	
	public double getVerticalLength(){
		vert = getLimetable().getEntry("tvert").getDouble(0);
		return vert;
	}

	public double getLong(){
		return getLimetable().getEntry("tlong").getDouble(0);
	}
	public double getShort(){
		return getLimetable().getEntry("tshort").getDouble(0);
	}

	/**
	 * 
	 * @param LEDState wanted LED state(0-on, 1-off, 2-blink)
	 */
	public void setLEDMode(int LEDState) {
		getLimetable().getEntry("ledMode").setDouble(LEDState);
	}
	
	/**
	 * Limelight Camera state
	 * @return Returns the current limelight camera state
	 */
	public double getCamMode() {
		camMode = getLimetable().getEntry("camMode").getDouble(0);
		return camMode;
	}

	public Target3D getCamTranslation() {
		double[] def = {-1,-1,-1,-1,-1,-1};
		double[] data = getLimetable().getEntry("camtran").getDoubleArray(def);
		Target3D targ =  new Target3D(new Vector3f(data[0]+X_OFFSET,-data[2],data[1]),new Vector3f(data[3],data[4],data[5]));
		longHistory.add(targ);
		if(checkLongHistory(targ))
			adjustedHistory.add(targ);

		if(adjustedHistory.size() > averageLength)
		adjustedHistory.remove(0);

		return targ;
	}

	public Target3D getAveragedCamTranslation(){
		getCamTranslation();
		Target3D average = new Target3D(0,0,0,0,0,0);
		for(Target3D d : adjustedHistory){
			average.translation.add(d.translation);
			average.rotation.add(d.rotation);
		}
		average.translation.multiplyScalar(1.0/averageLength);
		average.rotation.multiplyScalar(1.0/averageLength);
		return average;
	}

	public boolean checkLongHistory(Target3D in){
		double averageX = 0;
		for(Target3D d : longHistory){
			averageX += d.translation.x;
		}
		
		averageX /= longHistoryLength;

		return Math.abs(averageX - in.translation.x) > xTolerance;
	}
	/**
	 * get current pipeline that is being used
	 * @return Returns the current pipeline that is being used by the limelight
	 */
	public double getPipeline() {
		pipeline = getLimetable().getEntry("pipeline").getDouble(0);
		return pipeline;
	}
	
	/**
	 * Switches the current state of the LED between on and off;
	 */
	public void switchLED() {
		if(getLEDMode() == 0) {
			getLimetable().getEntry("ledMode").setDouble(1);
		}else if(getLEDMode() == 1) {
			getLimetable().getEntry("ledMode").setDouble(0);
		}else if(getLEDMode() == 2) {
			getLimetable().getEntry("ledMode").setDouble(1);
		}
	}
	
	/**
	 * Switches the camera mode between 0 and 1 (using vision processor or just make it a basic camera)
	 */
	public void switchCamera() {
		if(getCamMode() == 0) {
			getLimetable().getEntry("camMode").setDouble(1);
		}else if(getCamMode() == 1) {
			getLimetable().getEntry("camMode").setDouble(0);
		}
	}
	
	
	/**
	 * Set the pipeline
	 * @param pipeline Choose the current pipeline. You can setup pipelines in limelight.local:5801
	 */
	public void setPipeline(double pipeline) {
		getLimetable().getEntry("pipeline").setDouble(pipeline);
	}


}