package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Customlib.*;
import frc.robot.Customlib.MecanumDrive.NonNormalizedNumber;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.kauailabs.navx.frc.AHRS;
/**
 *
 */
public class DriveSubsystem extends Subsystem {

	//Speed Controllers
	public static Spark frontLeftMotor = new Spark(RobotMap.PWM.FRONT_LEFT_MOTOR_PORT);
	public static Spark frontRightMotor = new Spark(RobotMap.PWM.FRONT_RIGHT_MOTOR_PORT);
	public static Spark backLeftMotor = new Spark(RobotMap.PWM.BACK_LEFT_MOTOR_PORT);
	public static Spark backRightMotor = new Spark(RobotMap.PWM.BACK_RIGHT_MOTOR_PORT);
	
	//Mecanum Drive Variable... Used to move the robot
	public static MecanumDrive robotDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
	public static LimeLight limelight = new LimeLight();
	public static Encoder testEncoder = new Encoder(0,1);
	public static Arduino ard = new Arduino();
	public static boolean autoEnable = false;
	public static AHRS nav;
	public static double navAngle = 0;
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveCommand());
		 nav = new AHRS(SPI.Port.kMXP);
	}
	
	public static Spark getFrontLeftMotor() {
		return frontLeftMotor;
	}
	public static Spark getFrontRightMotor(){
		return frontRightMotor;
	}
	public static Spark getBackLeftMotor(){
		return backLeftMotor;
	}
	public static Spark getBackRightMotor(){
		return backRightMotor;
	}
    
	//Main Drive Function(called by DriveCommand)
    public static void mecanumDrive() {
    	robotDrive.drive(OI.getPlaystation());
	}
	public static void updateAngle(){
		navAngle = nav.getAngle() % 360;
		if(navAngle < 0) navAngle = 360 + navAngle;
		//System.out.println(navAngle);
		// String dta = ard.getData();
		// if(dta.charAt(dta.length()-1) == 'm')
		// System.out.println(dta);
		// System.out.println("Encoder: " + testEncoder.getRaw());
	}

	public static void focusTape(){
		//if(!limelight.getHasTarget()) return;
		double horiz = limelight.getHorizontalLength();
		double vert = limelight.getVerticalLength();
		double tLong = limelight.getLong();
		double tShort = limelight.getShort();


		double offset = limelight.getXOffset();
		double skewOff = limelight.getSkew();
		String ardData = ard.getData();

		System.out.println("Horiz: " + horiz + ",    Vert: " + vert + ",    Long: " + tLong + ",    Short: " + tShort);

		double distance = 0;
		try{
			distance = Double.parseDouble(ardData.substring(0, ardData.length()-1));
		}catch(Exception e){}
		//double z = Math.abs(skewOff) >= 0.5 ? 0.3*(skewOff/Math.abs(skewOff)) : 0;
		double x = 0;
		double y =0;
		if(limelight.getHasTarget())
		x = Math.abs(offset) >= 2.5 ? 0.7*(offset/Math.abs(offset)) : 0;

		//System.out.println(distance);
		y = Math.abs(distance) >= 30 ? .3 : 0;
		
		//System.out.println(x);
		double z = Math.abs(90-navAngle) >= 5 ? 0.6*((90-navAngle)/Math.abs(90-navAngle)) : 0;
		try{
			robotDrive.drive(x,y,z);
		}catch(NonNormalizedNumber e){
			System.err.println(e);
		}
		
	}

    
}

