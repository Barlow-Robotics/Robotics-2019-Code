package frc.robot.Customlib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Limelight library for easy use of the Limelight camera. Use it as you wish.
 */
public class LimeLight {		
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
		xOffset = getLimetable().getEntry("tx").getDouble(0);
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