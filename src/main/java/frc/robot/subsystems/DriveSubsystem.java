package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Customlib.LimeLight;
import frc.robot.Customlib.MecanumDrive;
import frc.robot.Customlib.MecanumDrive.NonNormalizedNumber;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
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
    public static boolean autoEnable = false;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveCommand());
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
	
	public static void focusTape(){
		if(!limelight.getHasTarget()) return;

		double offset = limelight.getXOffset();
		double skewOff = limelight.getSkew();
		System.out.println(skewOff);
		double z = Math.abs(skewOff) >= 0.5 ? 0.3*(skewOff/Math.abs(skewOff)) : 0;
		double x = Math.abs(offset) >= 2.5 ? 0.3*(offset/Math.abs(offset)) : 0;
		try{
			robotDrive.drive(x,0,z);
		}catch(NonNormalizedNumber e){
			System.err.println(e);
		}
		
	}

    
}

