/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  	public static final class PWM{
		//Drive
		public static final int FRONT_LEFT_MOTOR_PORT = 1; //2
		public static final int FRONT_RIGHT_MOTOR_PORT = 0; //3
		public static final int BACK_LEFT_MOTOR_PORT = 3; //0
		public static final int BACK_RIGHT_MOTOR_PORT = 2; //1
		//Other
		public static final int LIFT_MOTOR_PORT = 1;
		public static final int PLATFORM_MOTOR_PORT = 0;
  	}
  	public static final class DIO{
		  //Encoders
		public static final int[] frontLeftEncoderPorts = {2, 3}; //{4, 5}
		public static final int[] frontRightEncoderPorts = {0, 1}; //{6, 7}
		public static final int[] backLeftEncoderPorts = {6, 7}; //{0, 1}
		public static final int[] backRightEncoderPorts = {4, 5}; //{2, 3}
		//Hall effect sensors
		public static final int HES_F = 9;
		public static final int HES_B = 8;
	}
  	//CONTROLLER
	//Test
	public static final class Controllers{
		public static final int PLAYSTATION_PORT = 0;
		public static final int BOX_PORT = 1;
  	}
      //pneumatic ports
	  public static int openClawSolenoidPort = 0;
	  public static int closeClawSolenoidPort = 1;
	  public static int openExtendSolenoidPort = 2;
	  public static int closeExtendSolenoidPort = 3;
}
