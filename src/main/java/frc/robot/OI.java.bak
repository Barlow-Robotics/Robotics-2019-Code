/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public static double threshHoldValue = 0.1;
	//Playstation Controller Variable
  private static Joystick playstation = new Joystick(RobotMap.Controllers.PLAYSTATION_PORT);
  private static Joystick box = new Joystick(RobotMap.Controllers.BOX_PORT);
  //Logitech Controller
  // public static Joystick logitech = new Joystick(RobotMap.Controllers.LOGITECH_PORT);
  
  //Gyroscope Variable
  //public static AnalogGyro analogGyro = new AnalogGyro(RobotMap.GYROSCOPE_PORT);
  
  
  public static double getPlaystationX(){
    return playstation.getX();
  }
  public static double getPlaystationY(){
    return playstation.getY();
  }
  public static double getPlaystationZ(){
    return playstation.getZ();
  }
  public static double getThreshedPSX(){
    return threshold(playstation.getX(),threshHoldValue);
  }
  public static double getThreshedPSY(){
    return threshold(playstation.getY(),threshHoldValue);
  }
  public static double getThreshedPSZ(){
    return threshold(playstation.getZ(),threshHoldValue);
  }
  public static Joystick getPlaystation(){
    return playstation;
  }
  public static Joystick getBox(){
    return box;
  }
  public static double getBoxY(){
    return -box.getY();
  }
  private static double threshold(double in, double thresh){
		return Math.abs(in) > thresh ? inputFunction(in) : 0;
  }
  private static double inputFunction(double x){
    // if (Math.abs(x) < 0.4) {
    //   return x*0.01;
    // }
    return 1.25*(x-0.2); 
  }
}
