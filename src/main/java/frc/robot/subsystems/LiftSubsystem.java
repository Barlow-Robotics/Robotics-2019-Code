/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.LiftCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Spark liftMotor = new Spark(RobotMap.PWM.LIFT_MOTOR_PORT);
  public DigitalInput HES_F = new DigitalInput(RobotMap.DIO.HES_F);
  public DigitalInput HES_B = new DigitalInput(RobotMap.DIO.HES_B);
  public int command;
  public int location = -1;
  public int prevLocation = -1;
  public boolean goingUp = false;

  public void lift(){
    if(command != location){
      if(location < command){
        Robot.liftSubsystem.liftMotor.set(.5);
      }
      else if(location > command){
      Robot.liftSubsystem.liftMotor.set(-.3);
      }
    }
    else{
      Robot.liftSubsystem.liftMotor.set(0);
    }
  }
  public void setCommand(int newCommand){
    command = newCommand;
  }
  public void getLocation(){
    if(HES_F.get() && !HES_B.get()){
      location = -1;
    }
    else if(HES_F.get() && HES_B.get()){
      location = 0;
    }
    else if(!HES_F.get() && HES_B.get()){
      location = 1;
    }
  }
  //public static DigitalInput HES_L = new DigitalInput(RobotMap.DIO.HES_M);
  //public static DigitalInput HES_R = new DigitalInput(RobotMap.DIO.HES_T);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LiftCommand());
  }
}
