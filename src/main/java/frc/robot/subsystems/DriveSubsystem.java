package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Customlib.*;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.net.ServerSocket;
import edu.wpi.first.wpilibj.Spark;
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
	private final double MAX_ENCODER = 10000.0;

	// This constant represents the reading from the distance sensor that indicates the
	// bot is close enough to the target and we don't need to move any closer.
	private final int CLOSE_ENOUGH = 5 ; // wpk - place holder value for now

	// Distance where we really want to start slowing down
	private final int ALMOST_THERE = 10 ; // wpk - place holder value for now
	// low speed for final approach
	private final double FINAL_APPROACH_SPEED_FACTOR = 0.25 ; // wpk - place holder value for now

	// Distance at which we will start our approach
	private final int APPROACH_DISTANCE = 15 ; // wpk - place holder value for now
	// Speed we will use for start of our approach
	private final double APPROACH_SPEED_FACTOR = 0.5 ; // wpk - place holder value for now

	// Speed when outside approach distance
	private final double FULL_SPEED_FACTOR = 1.0 ;

	// Tolerence around centering bot to alignment line
	private final double ALIGNMENT_LINE_LATERAL_TOLERANCE = 10.0 ;  // wpk - need to figure out what a good value is.
	private final double ALIGNMENT_LINE_ANGULAR_TOLERANCE = 2.0 ; // wpk - need to figure out what a good value is
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
	private Spark frontLeftMotor; // = new Spark(RobotMap.PWM.FRONT_LEFT_MOTOR_PORT);
	private Spark frontRightMotor; // = new Spark(RobotMap.PWM.FRONT_RIGHT_MOTOR_PORT);
	private Spark backLeftMotor; // = new Spark(RobotMap.PWM.BACK_LEFT_MOTOR_PORT);
	private Spark backRightMotor; // = new Spark(RobotMap.PWM.BACK_RIGHT_MOTOR_PORT);

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

	// This is the initial heading recorded when the bot went into auto approach mode
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

		frontLeftMotor = new Spark(RobotMap.PWM.FRONT_LEFT_MOTOR_PORT);
		frontRightMotor = new Spark(RobotMap.PWM.FRONT_RIGHT_MOTOR_PORT);
		backLeftMotor = new Spark(RobotMap.PWM.BACK_LEFT_MOTOR_PORT);
		backRightMotor = new Spark(RobotMap.PWM.BACK_RIGHT_MOTOR_PORT);

		frontLeftMotor.setName("frontLeftMotor");
		frontRightMotor.setName("frontRightMotor");
		backLeftMotor.setName("backLeftMotor");
		backRightMotor.setName("backRightMotor");


		frontLeftEncoder = new Encoder(RobotMap.DIO.frontLeftEncoderPorts[0], RobotMap.DIO.frontLeftEncoderPorts[1]); 
		frontRightEncoder = new Encoder(RobotMap.DIO.frontRightEncoderPorts[0], RobotMap.DIO.frontRightEncoderPorts[1]);
		backLeftEncoder = new Encoder(RobotMap.DIO.backLeftEncoderPorts[0], RobotMap.DIO.backLeftEncoderPorts[1]); 
		backRightEncoder = new Encoder(RobotMap.DIO.backRightEncoderPorts[0], RobotMap.DIO.backRightEncoderPorts[1]); 
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
		// nav = new AHRS(SPI.Port.kMXP);

		driveMode = DriveMode.Manual;

	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveCommand());
	}



	public boolean isAutoAvailable() {
		// The logic here needs to look at the target areas returned by the limelight and
		// make a determination whether a target is visible (i.e., two inward facing target lines).
		// If there is a target visible, then this should return true, false otherwise.
		// wpk - you should think about the where to put the logic for determining if a target is visible.
		// If this is the only place you would ever want that, you could put it here. If you might want to
		// that from other classes (e.g., cueing the operator), consider putting it in the limelight class 
		// so its accessible elsewhere.
		return false; // TBD
	}


    private boolean isCloseEnough() {

		//wpk this function returns true if we have determined that the bot state can be 
		// changed from positioning to approach. This might be as simple as checking the range to the target.

		return false ;
	}



	// wpk - not sure who callls this, but it would be in response to the operator pressing
	// the auto button.
	public void SetModeAutoApproach() {

		if ( isAutoAvailable() ) {
			// need to set initial gyro and rotation values based on angle and distance to 
			// the target

			desiredTrackAngle = 0.0 ; // tbd need to compute angle relative to front of the bot
			
			startingHeading = nav.getFusedHeading() ;  // wpk need to make sure this is the right function to call

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
	}

	private boolean isInPosition() {
		// need to determine if we are close enough to the target and are aligned with it.

		// I recommend an approach such as :
		// if ( visionSystem.distanceToTarget() <= APPROACH_DISTANCE && visionSystem.alignmentLineIsVisible() ) {
		// 	return true ;
		// } else {
		// 	return false ;
		// }
			return false;
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
		SmartDashboard.putString("frontLeftEncoder", frontLeftEncoder.getRate()+"");
		SmartDashboard.putString("frontRightEncoder", frontRightEncoder.getRate()+"");
		SmartDashboard.putString("backRightEncoder", backRightEncoder.getRate()+"");
		SmartDashboard.putString("backLeftEncoder", backLeftEncoder.getRate()+"");
		SmartDashboard.putNumber("KP", KP);
		SmartDashboard.putNumber("KI", KI);
		SmartDashboard.putNumber("KD", KD);
		SmartDashboard.putBoolean("HEffect_F",Robot.liftSubsystem.HES_F.get());
		SmartDashboard.putBoolean("HEffect_B",Robot.liftSubsystem.HES_B.get());

		if(driveMode == null) driveMode = DriveMode.Manual;
        switch ( driveMode ) {
			
			case Manual :
				   // In this case, we just listen to the operator. 
				   // The following code was lifted from the original member function called mecanum()
				   // wpk - Do we need to cal drive cartesian here witrh X, Y, Z values derived from the
				   // joystick inputs?
				// wpk this line commented out to allow compile. 
				// robotDrive.drive(OI.getPlaystation());
				robotDrive.driveCartesian( OI.getThreshedPSX() , -OI.getThreshedPSY(), OI.getThreshedPSZ());
			    break ;

			case Positioning :

                if ( isInPosition() ) {
					//driveMode = DriveMode.Approaching ;
				} else {
					// need to compute angle from front of bot to target so that we can make sure the
					// target is centered in the limelight view.
					LimeLight.Target3D targ = visionSystem.limeLight.getCamTranslation();
					double xOff = visionSystem.limeLight.getXOffset();
					double zRotation =  Math.abs(xOff) > 10 ? 0.5*xOff/Math.abs(xOff) : 0;
					double xMove = xOff > 2 ? targ.translation.x/Math.abs(targ.translation.x) : 0;


					//VisionSystem.BearingData b = visionSystem.bearingToTarget();


					// convert the angle to a rotation input to the mecanum drive


					// Once this angle is computed, the desired track will be updated so that the bot
					// will move along the desired track after rotating the bot to center the target in the
					// limelight field of view (FOV)
					//desiredTrackAngle = desiredTrackAngle + b.angle;  // should this be add or subtract?

					// wpk - need to think about the above line. Worried that its going to sum the change frame to frame
					// which is not what we want. May need to record angle from nav x at start?

					robotDrive.driveCartesian( xMove, 0.0, zRotation, 0 ) ;
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

					// need to fill in the calculation that computes the X distance from the center of the line
					// to the center line of the screen.
					double distanceFromCenter = 0.0 ; // tbd 
					double angleOfAlignmentLine = 0.0 ; // tbd

					double xSpeed = 0.0 ;
					double ySpeed = 0.0 ;
					double zRotation = 0.0 ;

					if ( Math.abs(distanceFromCenter) > ALIGNMENT_LINE_LATERAL_TOLERANCE) {
						// The bot needs to be moved left or right
						xSpeed = 0.0 ; // wpk tbd - need to figure out speed based on distance from line
						// fix the sign of the speed command so we go the right direction
						xSpeed = xSpeed * (distanceFromCenter/Math.abs(distanceFromCenter)) ;
					} else {
						// the bot is close enough to the alignment line, so lets make sure its pointing at the target
						if ( Math.abs( angleOfAlignmentLine ) > ALIGNMENT_LINE_ANGULAR_TOLERANCE ) {
							zRotation = ALIGNMENT_ROTATION_SPEED * (distanceFromCenter/Math.abs(distanceFromCenter)) ;
						}
					}

					ySpeed = approachSpeedFactorToTarget() ;

					robotDrive.driveCartesian( ySpeed, xSpeed, zRotation, 0.0 ) ;

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

	// public static Spark getFrontLeftMotor() {
	// return frontLeftMotor;
	// }

	// public static Spark getFrontRightMotor() {
	// return frontRightMotor;
	// }

	// public static Spark getBackLeftMotor() {
	// return backLeftMotor;
	// }

	// public static Spark getBackRightMotor() {
	// return backRightMotor;
	// }

	// // Main Drive Function(called by DriveCommand)
	// public static void mecanumDrive() {
	// robotDrive.drive(OI.getPlaystation());
	// }


    private double getCompassHeading() {
		double navAngle = nav.getAngle() % 360;
		if (navAngle < 0)
			navAngle = 360 + navAngle;
		return navAngle ;
	}

	// public void updateAngle() {
	// 	navAngle = nav.getAngle() % 360;
	// 	if (navAngle < 0)
	// 		navAngle = 360 + navAngle;
	// 	// System.out.println(navAngle);
	// 	// String dta = ard.getData();
	// 	// if(dta.charAt(dta.length()-1) == 'm')
	// 	// System.out.println(dta);
	// 	// System.out.println("Encoder: " + testEncoder.getRaw());
	// }



	// wpk the logic in here should be moved into the target detector
	// commented out for now to address compiler errors

	// public static void focusTape() {
	// 	// if(!limelight.getHasTarget()) return;
	// 	double horiz = limelight.getHorizontalLength();
	// 	double vert = limelight.getVerticalLength();
	// 	double tLong = limelight.getLong();
	// 	double tShort = limelight.getShort();

	// 	double offset = limelight.getXOffset();
	// 	double skewOff = limelight.getSkew();
	// 	String ardData = ard.getData();

	// 	System.out.println("Horiz: " + horiz + ",    Vert: " + vert + ",    Long: " + tLong + ",    Short: " + tShort);

	// 	double distance = 0;
	// 	try {
	// 		distance = Double.parseDouble(ardData.substring(0, ardData.length() - 1));
	// 	} catch (Exception e) {
	// 	}
	// 	// double z = Math.abs(skewOff) >= 0.5 ? 0.3*(skewOff/Math.abs(skewOff)) : 0;
	// 	double x = 0;
	// 	double y = 0;
	// 	if (limelight.getHasTarget())
	// 		x = Math.abs(offset) >= 2.5 ? 0.7 * (offset / Math.abs(offset)) : 0;

	// 	// System.out.println(distance);
	// 	Mat frame = new Mat();

	// 	CameraServer.getInstance().getVideo("LimeLight").grabFrame(frame);

	// 	y = Math.abs(distance) >= 30 ? .3 : 0;

	// 	// System.out.println(x);
	// 	double z = 0.3 * calcAngle(frame);
	// 	try {
	// 		robotDrive.drive(x, y, z);
	// 	} catch (NonNormalizedNumber e) {
	// 		System.err.println(e);
	// 	}

	// }

	// public static int calcAngle(Mat input) {
	// 	gripz.process(input);
	// 	ArrayList<MatOfPoint> contours = gripz.filterContoursOutput();
	// 	ArrayList<Rect> out = new ArrayList<Rect>();
	// 	for (MatOfPoint cont : contours)
	// 		out.add(Imgproc.boundingRect(cont));

	// 	ArrayList<Rect> sortedOut = new ArrayList<Rect>();

	// 	for (Rect rect : out) {
	// 		if (sortedOut.size() == 0) {
	// 			sortedOut.add(rect);
	// 			continue;
	// 		}
	// 		for (int i = 0; i < sortedOut.size(); i++) {
	// 			if (sortedOut.get(i).x < rect.x) {
	// 				sortedOut.add(i, rect);
	// 				continue;
	// 			}
	// 			sortedOut.add(rect);
	// 		}
	// 	}

	// 	double areaR = sortedOut.get(0).y * sortedOut.get(0).x;
	// 	double areaL = sortedOut.get(sortedOut.size() - 1).y * sortedOut.get(sortedOut.size() - 1).x;
	// 	return (Math.abs(areaR - areaL) >= 100) ? -1 : 1;
	// }

}
