/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;

/**
 * Add your docs here.
 */
public class PlatformSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  double defaultHeight = 0;
  double platFormHeight = 0;
  double heightGroundAdjustment = 5;
  double heightPlatormAdjustment = 10;
  
  public static Solenoid openSolenoidBack = new Solenoid(4);
	public static Solenoid closeSolenoidBack = new Solenoid(5);
  public static Solenoid openSolenoidFront = new Solenoid(6);
  public static Solenoid closeSolenoidFront = new Solenoid(7);
  public static Spark habitatMotor = new Spark(RobotMap.PWM.PLATFORM_MOTOR_PORT);

  public static boolean openBack = false;
  public static boolean openFront = false;
  @Override
  public void initDefaultCommand() {
    
  }
  public void toggleFront(){
    openFront = !openFront;
    updateFrontSolenoids();
  }
  public void drive(double speed){
    habitatMotor.set(speed);
  }
  public void toggleBack(){
    openBack = !openBack;
    updateBackSolenoids();
  }
  private void updateBackSolenoids() {
    openSolenoidBack.set(openBack);
    closeSolenoidBack.set(!openBack);
  }
  private void updateFrontSolenoids() {
    openSolenoidFront.set(openFront);
    closeSolenoidFront.set(!openFront);
  }
  public void checkHeight(){
    // if(sensor1-heightGroundAdjustment > defaultHeight && sensor2-heightGroundAdjustment > defaultHeight){
    //     if(sensor-10 >= platFormHeight){
    //       //activate motor 
    //     }
    // }
  }
}
