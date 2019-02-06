/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

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
  @Override
  public void initDefaultCommand() {
  }

  public void checkHeight(){
    // if(sensor1-heightGroundAdjustment > defaultHeight && sensor2-heightGroundAdjustment > defaultHeight){
    //     if(sensor-10 >= platFormHeight){
    //       //activate motor 
    //     }
    // }
  }
}
