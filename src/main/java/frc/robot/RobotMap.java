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
		public static final int FRONT_LEFT_MOTOR_PORT = 4;
		public static final int FRONT_RIGHT_MOTOR_PORT = 5;
		public static final int BACK_LEFT_MOTOR_PORT = 7;
		public static final int BACK_RIGHT_MOTOR_PORT = 8;
  }
	  public static final int openArmSolenoidPort = 0;
	  public static final int closeArmSolenoidPort = 1;
  	//CONTROLLER
	public static final class Controllers{
    public static final int PLAYSTATION_PORT = 0;
  }
}
