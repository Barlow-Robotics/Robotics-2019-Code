package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Customlib.*;
import frc.robot.Customlib.LimeLight.Target3D;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.net.ServerSocket;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.kauailabs.navx.frc.AHRS;
import org.opencv.core.*;

/**
 *
 */
public class DriveSubsystem extends Subsystem {


	////////////////////
	// Constants
	////////////////////

	// wpk - 
	// This value is a percentage of the maximum value read from an encoder
	// when the slowest motor is driven to 100%. This number needs to account for
	// reduction in motor speed when the battery starts to run down, as well as the
	// fact that the PID controller may add some gain beyond the set point if the motor needs
	// to "catch up" to the set value.
	private final double MAX_ENCODER = 1000.0;
	private final double DEBUG_MULTIPLIER = 0.3;
	// This constant represents the reading from the distance sensor that indicates the
	// bot is close enough to the target and we don't need to move any closer.
	private final int CLOSE_ENOUGH = -3 ; // wpk - place holder value for now

	// Distance where we really want to start slowing down
	private final int ALMOST_THERE = 10 ; // wpk - place holder value for now
	// low speed for final approach
	private final double FINAL_APPROACH_SPEED_FACTOR = 0.25 ; // wpk - place holder value for now

	// Distance at which we will start our approach
	private final double APPROACH_DISTANCE = 30.0 ; // wpk - place holder value for now
	// Speed we will use for start of our approach
	private final double APPROACH_SPEED_FACTOR = 0.5 ; // wpk - place holder value for now


	// Speed when outside approach distance
	private final double FULL_SPEED_FACTOR = 1.0 ;

	// Tolerence around centering bot to alignment line
	private final double ALIGNMENT_LINE_LATERAL_TOLERANCE = 10.0 ;  // wpk - need to figure out what a good value is.
	private final double ALIGNMENT_LINE_ANGULAR_TOLERANCE = 6.0 ; // wpk - need to figure out what a good value is
	private final double ALIGNMENT_ROTATION_SPEED = 0.1 ; // wpk - need to figure out what a good value is

	// These are the proportional, integral, and derivative coefficients used in the
	// PID control.
	public double KP = 0.00002; // update when PID is tuned
	public double KI = 0.0; // update when PID is tuned
	public double KD = 0.000000005; // update when PID is tuned

	// This is the feed forward term used in the PID controller. For understanding
	// what this is for,
	// consult the WPILib docs on feed forward or look at the PIDBase code.
	private final double KF = 1.0 / MAX_ENCODER;


    ////////////////////
	// Member variables
	////////////////////

	// Speed Controllers for the motors
	private VictorSP frontLeftMotor; // = new Spark(RobotMap.PWM.FRONT_LEFT_MOTOR_PORT);
	private VictorSP frontRightMotor; // = new Spark(RobotMap.PWM.FRONT_RIGHT_MOTOR_PORT);
	private VictorSP backLeftMotor; // = new Spark(RobotMap.PWM.BACK_LEFT_MOTOR_PORT);
	private VictorSP backRightMotor; // = new Spark(RobotMap.PWM.BACK_RIGHT_MOTOR_PORT);

	// Encoders for each wheel
	private Encoder frontLeftEncoder;
	private Encoder frontRightEncoder;
	private Encoder backLeftEncoder;
	private Encoder backRightEncoder;

	// PID Speed controllers used to ensure each wheel rotates at a specific
	// commanded speed.
	private PIDSpeedCtrl frontLeftSpeedCtrl;
	private PIDSpeedCtrl frontRightSpeedCtrl;
	private PIDSpeedCtrl backLeftSpeedCtrl;
	private PIDSpeedCtrl backRightSpeedCtrl;

	// Mecanum Drive Variable... Used to move the robot
	private MecanumDrive robotDrive ;
	ServerSocket outputSocket;
	// 
	private VisionSystem visionSystem = new VisionSystem();
	
	// private LimeLight limelight;
	// private Encoder testEncoder;
	// private Arduino ard;
	// private GripPipeline gripz;
	// private Webcam alignmentCam ;  // This refers to the web cam used to pick up the alignment lines.

	private boolean autoEnable = false;
	private AHRS nav;
	private double navAngle = 0;

	// This is the direction of travel we want the bot to follow when in positioning mode
	private double desiredTrackAngle;

	// This is the
	// initial heading recorded when the bot went into auto approach mode
	private double startingHeading ;

	enum DriveMode {
		Manual, Positioning, Approaching
	};

	DriveMode driveMode;
	// public class MotorIntercept extends Spark{
	// 	public double lastSetSpeed = 0;
	// 	public MotorIntercept(int port){
	// 		super(port);
	// 	}

	// 	@Override
	// 	public void setSpeed(double speed){
	// 		super.setSpeed(OI.getThreshedPSX() != 0 ||
	// 			 OI.getThreshedPSY() != 0 || OI.getThreshedPSZ() != 0
	// 			 ? speed : 0);
	// 	}
	// 	private double threshHold(double in, double thresh){
	// 		return Math.abs(in) > thresh ? in : 0;
	// 	}
	// }
	
	public DriveSubsystem() {

		frontLeftMotor = new VictorSP(RobotMap.PWM.FRONT_LEFT_MOTOR_PORT);
		frontRightMotor = new VictorSP(RobotMap.PWM.FRONT_RIGHT_MOTOR_PORT);
		backLeftMotor = new VictorSP(RobotMap.PWM.BACK_LEFT_MOTOR_PORT);
		backRightMotor = new VictorSP(RobotMap.PWM.BACK_RIGHT_MOTOR_PORT);

		frontLeftMotor.setName("frontLeftMotor");
		frontRightMotor.setName("frontRightMotor");
		backLeftMotor.setName("backLeftMotor");
		backRightMotor.setName("backRightMotor");


		frontLeftEncoder = new Encoder(RobotMap.DIO.frontLeftEncoderPorts[0], RobotMap.DIO.frontLeftEncoderPorts[1]); 
		frontRightEncoder = new Encoder(RobotMap.DIO.frontRightEncoderPorts[0], RobotMap.DIO.frontRightEncoderPorts[1]);
		backLeftEncoder = new Encoder(RobotMap.DIO.backLeftEncoderPorts[0], RobotMap.DIO.backLeftEncoderPorts[1]); 
		backRightEncoder = new Encoder(RobotMap.DIO.backRightEncoderPorts[0], RobotMap.DIO.backRightEncoderPorts[1]); 
		frontLeftEncoder.reset();
		frontLeftEncoder.setName("FL encoder");

		frontRightEncoder.reset();
		frontRightEncoder.setName("FR encoder");
		
		backLeftEncoder.reset();
		backLeftEncoder.setName("BL encoder");

		backRightEncoder.reset();
		backRightEncoder.setName("BR encoder");

		// frontRightEncoder.setReverseDirection(true);
		// backRightEncoder.setReverseDirection(true);

		frontLeftSpeedCtrl = new PIDSpeedCtrl(KP, KI, KD, KF, frontLeftEncoder, frontLeftMotor, 14572);
		frontRightSpeedCtrl = new PIDSpeedCtrl(KP, KI, KD, KF, frontRightEncoder, frontRightMotor,14573);
		backLeftSpeedCtrl = new PIDSpeedCtrl(KP, KI, KD, KF, backLeftEncoder, backLeftMotor,14574);
		backRightSpeedCtrl = new PIDSpeedCtrl(KP, KI, KD, KF, backRightEncoder, backRightMotor,14575);

		robotDrive = new MecanumDrive(frontLeftSpeedCtrl, backLeftSpeedCtrl, frontRightSpeedCtrl, backRightSpeedCtrl);
		robotDrive.setMaxOutput(MAX_ENCODER);

		// limelight = new LimeLight();
		// alignmentCam = new WebCam() ;
		// ard = new Arduino();
		// gripz = new GripPipeline();
		nav = new AHRS(Port.kMXP);

		driveMode = DriveMode.Manual;

	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveCommand());
	}


	public static final int TOO_CLOSE = 10;
	public boolean isAutoAvailable() {
		// The logic here needs to look at the target areas returned by the limelight and
		// make a determination whether a target is visible (i.e., two inward facing target lines).
		// If there is a target visible, then this should return true, false otherwise.
		// wpk - you should think about the where to put the logic for determining if a target is visible.
		// If this is the only place you would ever want that, you could put it here. If you might want to
		// that from other classes (e.g., cueing the operator), consider putting it in the limelight class 
		// so its accessible elsewhere.
		boolean aLVisible = visionSystem.alignmentLineIsVisible();
		boolean hasTarget = visionSystem.limeLight.getHasTarget();
		double dist = visionSystem.distanceToTarget();
		SmartDashboard.putBoolean("aLVisible", aLVisible);
		SmartDashboard.putBoolean("hasTarget", hasTarget);
		SmartDashboard.putNumber("Targ Dist", dist);
		return (aLVisible && dist < TOO_CLOSE) || hasTarget; // TBD
	}

	
    private boolean isCloseEnough() {

		//wpk this function returns true if we have determined that the bot state can be 
		// changed from positioning to approach. This might be as simple as checking the range to the target.

		return visionSystem.distanceToTarget() < CLOSE_ENOUGH;
	}



	// wpk - not sure who callls this, but it would be in response to the operator pressing
	// the auto button.

	public double startBearing;
	public void SetModeAutoApproach() {

		if ( isAutoAvailable() ) {
			// need to set initial gyro and rotation values based on angle and distance to 
			// the target

			 
			startingHeading = nav.getAngle() ;
			double bearing = visionSystem.bearingToTarget().angle;  // wpk need to make sure this is the right function to call
			startBearing = bearing;
            double bearingInRadians = Math.toRadians(bearing);

			double distanceToTarget = visionSystem.distanceToTarget() ;
			double distanceToApproach 
			   = Math.sqrt(distanceToTarget*distanceToTarget
						   + APPROACH_DISTANCE*APPROACH_DISTANCE
						   - 2.0*APPROACH_DISTANCE*visionSystem.distanceToTarget()*Math.cos(bearingInRadians)) ;
			


			desiredTrackAngle = Math.toDegrees(Math.asin(APPROACH_DISTANCE*Math.sin(bearingInRadians)/distanceToApproach) );

			driveMode = DriveMode.Positioning ;
		}
	}

	public void SetModeManual() {
		driveMode = DriveMode.Manual;
		// stop the bot
		// robotDrive.driveCartesian( OI.getPlaystationX(), OI.getPlaystationY(),
		// OI.getPlaystationZ());
		
		// Reset the PID controllers to get rid of built up integral error
		// frontLeftSpeedCtrl.reset() ;
		// frontRightSpeedCtrl.reset() ;
		// backLeftSpeedCtrl.reset() ;
		// backRightSpeedCtrl.reset() ;
	}
	public void setModePositioning() {
		driveMode = DriveMode.Positioning;
		System.out.println("In positioning mode");
	}

	private boolean isInPosition() {
		// need to determine if we are close enough to the target and are aligned with it.

		// I recommend an approach such as :
		if ( visionSystem.distanceToTarget() <= APPROACH_DISTANCE && visionSystem.alignmentLineIsVisible() ) {
			return true ;
		} else {
			return false ;
		}
	}





	private double approachSpeedFactorToTarget() {

		// Simple aproach applied here where a schedule of speeds is defined
		// based on distance to the destination. Faster further out, slower as
		// the bot gets closer.

		double speed = 0.0 ;
		if ( visionSystem.distanceToTarget() > CLOSE_ENOUGH && visionSystem.distanceToTarget() <= ALMOST_THERE ) {
			speed = FINAL_APPROACH_SPEED_FACTOR ;
		} else if (visionSystem.distanceToTarget() > ALMOST_THERE && visionSystem.distanceToTarget() <= APPROACH_DISTANCE) {
			speed = APPROACH_SPEED_FACTOR ;
		} else {
			speed = FULL_SPEED_FACTOR ;
		}
		return speed ;

	}

	// Need to understand where this can be called from. Maybe command?
	public void doDriving() {
		visionSystem.updateVision();
		SmartDashboard.putBoolean("AutoEnabled", isAutoAvailable());
		SmartDashboard.putNumber("NavAngle",nav.getAngle());
		SmartDashboard.putString("Drive mode", driveMode.name());
		System.out.println("frontLeftEncoder" + frontLeftEncoder.getRate());
		System.out.println("frontRightEncoder"+ frontRightEncoder.getRate());
		System.out.println("backRightEncoder"+ backRightEncoder.getRate());
		System.out.println("backLeftEncoder"+ backLeftEncoder.getRate());
		SmartDashboard.putString("frontLeftEncoder", frontLeftEncoder.getRate()+"");
		SmartDashboard.putString("frontRightEncoder", frontRightEncoder.getRate()+"");
		SmartDashboard.putString("backRightEncoder", backRightEncoder.getRate()+"");
		SmartDashboard.putString("backLeftEncoder", backLeftEncoder.getRate()+"");
		SmartDashboard.putNumber("KP", KP);
		SmartDashboard.putNumber("KI", KI);
		SmartDashboard.putNumber("KD", KD);
		SmartDashboard.putNumber("LIDAR: ", visionSystem.server.getLastPacket().lidarDist);
		if(driveMode == null) driveMode = DriveMode.Manual;
		VisionSystem.BearingData b = visionSystem.bearingToTarget();
		SmartDashboard.putNumber("Bearing",b.angle);


        switch ( driveMode ) {
			
			case Manual :
				   // In this case, we just listen to the operator. 
				   // The following code was lifted from the original member function called mecanum()
				   // wpk - Do we need to cal drive cartesian here witrh X, Y, Z values derived from the
				   // joystick inputs?
				// wpk this line commented out to allow compile. 
				// robotDrive.drive(OI.getPlaystation());
				boolean auto = OI.getPlaystation().getRawButton(3);
				double xSpeed = OI.getThreshedPSX();
				double zSpeed = OI.getThreshedPSZ();
				Target3D targ = visionSystem.limeLight.getCamTranslation();
				double yRotation = visionSystem.limeLight.getXOffset();
				if(auto){
					if(!visionSystem.limeLight.getHasTarget()){
						//TODO alignment line code
						xSpeed = 0;
						zSpeed = 0;
						// if(visionSystem.getAlignmentRectangle() == null) break;
						// Point[] points = new Point[4];
					
						// visionSystem.getAlignmentRectangle().points(points);
						// RotatedRect aL = visionSystem.getAlignmentRectangle();
						// double aLX = aL.center.x;
						// double aLRot = points[1].y < points[3].y ? aL.angle : 90 + aL.angle;
						// if(Math.abs(aLX) > 0.3){
						// 	xSpeed = 0.2*aLX/Math.abs(aLX);
						// }
						
						// if(Math.abs(aLRot) > 0.3){
						// 	zSpeed = 0.2*aLRot/Math.abs(aLRot);
						// }

					}else{
						if(Math.abs(yRotation) >= ALIGNMENT_LINE_ANGULAR_TOLERANCE){
							zSpeed = ALIGNMENT_ROTATION_SPEED*yRotation/Math.abs(yRotation);
						}else{
							zSpeed = 0;
						}
						if(Math.abs(targ.translation.x) > 2){
							xSpeed = -0.3*targ.translation.x/Math.abs(targ.translation.x);
						}else{
							xSpeed = 0;
						}
					}
				}
				robotDrive.driveCartesian(xSpeed, visionSystem.server.getLastPacket().lidarDist > 4 ? -OI.getThreshedPSY() : 0, zSpeed);
			    break ;

			case Positioning :

                if ( isInPosition() ) {
					driveMode = DriveMode.Approaching ;
				} else {
					// need to compute angle from front of bot to target so that we can make sure the
					// target is centered in the limelight view.
					// LimeLight.Target3D targ = visionSystem.limeLight.getCamTranslation();
					// double xOff = visionSystem.limeLight.getXOffset();
					// double zRotation =  Math.abs(xOff) > 10 ? 0.5*xOff/Math.abs(xOff) : 0;
					// double xMove = xOff > 2 ? targ.translation.x/Math.abs(targ.translation.x) : 0;



					System.out.println("Bearing: " + b.angle + "\tTrack: " + desiredTrackAngle + "\t SNavHead: " + startingHeading + "\t CNavHead: " + nav.getAngle() + "\t Dist: " + visionSystem.distanceToTarget());

					// convert the angle to a rotation input to the mecanum drive


					// Once this angle is computed, the desired track will be updated so that the bot
					// will move along the desired track after rotating the bot to center the target in the
					// limelight field of view (FOV)
					//desiredTrackAngle += startingHeading - nav.getAngle();  // should this be add or subtract?

					double deltaTrack = desiredTrackAngle + startingHeading-nav.getAngle() ;
					// wpk - need to think about the above line. Worried that its going to sum the change frame to frame
					// which is not what we want. May need to record angle from nav x at start?
					double zRotation = Math.abs(visionSystem.limeLight.getXOffset()) > 1 ?
						 visionSystem.limeLight.xOffset/Math.abs(visionSystem.limeLight.xOffset)*0.1 : 0;
					robotDrive.driveCartesian(0.0, 1.0*DEBUG_MULTIPLIER,
					 zRotation, //TODO add this in
					 //90.0) ;
					 deltaTrack) ;
			    }
			    break ;

			case Approaching :

				if ( isCloseEnough() ) {
					driveMode = DriveMode.Manual ;
				} else {
					// get the rotated rectangle for the alignment line from the web cam video
					// if the center of the rotated rectangle is not close enough of the center of
					// the robot (as computed from the cam image relative to the bot), then move the 
					// bot left or right to compensate.
					// note: we'll need to consider how fast to move the bot in these circumstances to make
					// sure we don't overshoot the line. Maybe use a speed schedule?

					// On the other hand, if the bot is aligned left right with the alignment line,
					// then adjust the bot's orientation to keep the angle of the line relative to the bot near zero
					// as we drive in.

					// wpk - some of this code doesn't yet exist and will need to be created
					RotatedRect alignmentLineRectangle = visionSystem.getAlignmentRectangle() ;
					double distanceFromCenter = 0;
					
					if(alignmentLineRectangle == null){
						driveMode = DriveMode.Manual;
						return;
					} 
					// need to fill in the calculation that computes the X distance from the center of the line
					// to the center line of the screen.
					distanceFromCenter = alignmentLineRectangle.center.x - 320/2 + 33; // tbd
					SmartDashboard.putNumber("Target Dist From Center",distanceFromCenter);

					Point[] points = new Point[4];
					
					alignmentLineRectangle.points(points); 
					double angleOfAlignmentLine = points[1].y < points[3].y ?
						alignmentLineRectangle.angle : 90 + alignmentLineRectangle.angle;
					
					// double xSpeed = 0.0 ;
					double ySpeed = 0.0 ;
					double zRotation = 0.1 * (Math.abs(angleOfAlignmentLine) > ALIGNMENT_LINE_ANGULAR_TOLERANCE ?
						angleOfAlignmentLine/Math.abs(angleOfAlignmentLine) : 0);
					if ( Math.abs(distanceFromCenter) > ALIGNMENT_LINE_LATERAL_TOLERANCE) {
						// The bot needs to be moved left or right
						// fix the sign of the speed command so we go the right direction
						xSpeed = 0.1 * (-distanceFromCenter/Math.abs(distanceFromCenter)) ;
					}
					ySpeed = approachSpeedFactorToTarget();
					// System.out.println("zRotation: " + zRotation + " xSpeed: " + xSpeed + " distance: " + distanceFromCenter + " angle: " + angleOfAlignmentLine);

					robotDrive.driveCartesian(0,0,0,0);
					// robotDrive.driveCartesian( xSpeed, 0, zRotation) ;

				}
			    break ;

		}

	}
	// Graph of this ramp function: https://www.desmos.com/calculator/xal57r1qdk
	public double approachFunction(double angle){
		return (-Math.pow(1.2,-Math.abs(angle))+1)*angle/Math.abs(angle);
	}

	public double threshHold(double in, double thresh){
		return Math.abs(in) > thresh ? in : 0;
	}

    private double getCompassHeading() {
		double navAngle = nav.getAngle() % 360;
		if (navAngle < 0)
			navAngle = 360 + navAngle;
		return navAngle ;
	}

}
