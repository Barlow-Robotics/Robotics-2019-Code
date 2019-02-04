package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Customlib.*;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.kauailabs.navx.frc.AHRS;
import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import org.opencv.core.Mat;

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
	private final double MAX_ENCODER = 1.0;  // wpk - place holder value for now

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
	private final double KP = 0; // update when PID is tuned
	private final double KI = 0; // update when PID is tuned
	private final double KD = 0; // update when PID is tuned

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

	// 
	private VisionSystem visionSystem ;
	
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

	class getData extends Spark{
        public getData(int channel){
            super(channel);
        }

        @Override
        public void pidWrite(double output) {
            super.pidWrite(output);
            System.out.println("PIDWRITE: " + output);
        }
	}
	
	public DriveSubsystem() {

		frontLeftMotor = new getData(RobotMap.PWM.FRONT_LEFT_MOTOR_PORT);
		frontRightMotor = new getData(RobotMap.PWM.FRONT_RIGHT_MOTOR_PORT);
		backLeftMotor = new getData(RobotMap.PWM.BACK_LEFT_MOTOR_PORT);
		backRightMotor = new getData(RobotMap.PWM.BACK_RIGHT_MOTOR_PORT);

		frontLeftMotor.setName("frontLeftMotor");
		frontRightMotor.setName("frontRightMotor");
		backLeftMotor.setName("backLeftMotor");
		backRightMotor.setName("backRightMotor");

		frontLeftMotor.setInverted(true);
		backLeftMotor.setInverted(true);

		/**
		 * TODO add these to robot map
		 */
		frontLeftEncoder = new Encoder(4, 5); 
		frontRightEncoder = new Encoder(2, 3);
		backLeftEncoder = new Encoder(0, 1); 
		backRightEncoder = new Encoder(6, 7); 
		
		frontLeftSpeedCtrl = new PIDSpeedCtrl(KP, KI, KD, KF, frontLeftEncoder, frontLeftMotor);
		frontRightSpeedCtrl = new PIDSpeedCtrl(KP, KI, KD, KF, frontRightEncoder, frontRightMotor);
		backLeftSpeedCtrl = new PIDSpeedCtrl(KP, KI, KD, KF, backLeftEncoder, backLeftMotor);
		backRightSpeedCtrl = new PIDSpeedCtrl(KP, KI, KD, KF, backRightEncoder, backRightMotor);

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
		SmartDashboard.putNumber("frontLeftEncoder", frontLeftEncoder.getRate());
		SmartDashboard.putNumber("frontRightEncoder", frontRightEncoder.getRate());
		SmartDashboard.putNumber("backRightEncoder", backRightEncoder.getRate());
		SmartDashboard.putNumber("backLeftEncoder", backLeftEncoder.getRate());

		if(driveMode == null) driveMode = DriveMode.Manual;
        switch ( driveMode ) {
			
			case Manual :
				   // In this case, we just listen to the operator. 
				   // The following code was lifted from the original member function called mecanum()
				   // wpk - Do we need to cal drive cartesian here witrh X, Y, Z values derived from the
				   // joystick inputs?
				// wpk this line commented out to allow compile. 
				// robotDrive.drive(OI.getPlaystation());
				robotDrive.driveCartesian( 0, OI.getPlaystationY(), 0);
			    break ;

			case Positioning :

                if ( isInPosition() ) {
					driveMode = DriveMode.Approaching ;
					desiredTrackAngle = visionSystem.bearingToTarget() ;

				} else {
					// need to compute angle from front of bot to target so that we can make sure the
					// target is centered in the limelight view.
					double angleFromCenter = 0.0 ; // tbd

					// convert the angle to a rotation input to the mecanum drive
					double zRotation = 0.0 ;  // tbd

					// Once this angle is computed, the desired track will be updated so that the bot
					// will move along the desired track after rotating the bot to center the target in the
					// limelight field of view (FOV)
					desiredTrackAngle = desiredTrackAngle + angleFromCenter ;  // should this be add or subtract?

					// wpk - need to think about the above line. Worried that its going to sum the change frame to frame
					// which is not what we want. May need to record angle from nav x at start?

					robotDrive.driveCartesian( 0.0, 1.0, zRotation, desiredTrackAngle ) ;
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
