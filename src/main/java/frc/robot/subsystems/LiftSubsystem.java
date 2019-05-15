/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public enum CommandEnum {Bottom, Middle, Top, None};
  enum StateEnum {Bottom, BottomToMiddle, Middle, MiddleToBottomleavingMiddle, MiddleToBottomWaitingBottom,  MiddleToTopLeavingMiddle, MiddleToTopWaitingTop, Top, TopToMiddle, StartState};
  public CommandEnum commandedState = CommandEnum.Bottom;
  public StateEnum currentState = StateEnum.StartState;
  private final double UP_SPEED = 0.7;
  private final double DOWN_SPEED = -0.3;
  private final double STALL_SPEED = .3;
  
  public void lift(){
    SmartDashboard.putString("Current State", currentState.name());
    if(OI.getBox().getRawAxis(3) != 1){
      if(!Robot.armSubsystem.openExtend){
    switch ( currentState ){
      case StartState:
        liftMotor.set(UP_SPEED);
        if(getLocation() == CommandEnum.Bottom){
          liftMotor.set(STALL_SPEED);
          currentState = StateEnum.Bottom;
        }
        else if(getLocation() == CommandEnum.Middle || getLocation() == CommandEnum.Top){
          liftMotor.set(DOWN_SPEED);
        }
        
      case Bottom:
         if ( commandedState != CommandEnum.Bottom ) {
          liftMotor.set(UP_SPEED ) ;
          currentState = StateEnum.BottomToMiddle ;
         }
         else{
           liftMotor.set(0.0);
         }
         break ;
      case BottomToMiddle:
         if( commandedState == CommandEnum.Bottom ){
           liftMotor.set(DOWN_SPEED);
           currentState = StateEnum.MiddleToBottomleavingMiddle;
         }
         else if( commandedState == CommandEnum.Middle){
          liftMotor.set(UP_SPEED);
         }
         if ( getLocation() == CommandEnum.Middle) {
             currentState = StateEnum.Middle ;
         }
         //else if( getLocation() == CommandEnum.Bottom){
         //  currentState = StateEnum.Bottom;
         //}
         break ;
      case Middle:
         if ( commandedState == CommandEnum.Bottom) {
             liftMotor.set(DOWN_SPEED) ;
             currentState = StateEnum.MiddleToBottomleavingMiddle;
         } else if ( commandedState == CommandEnum.Top) {
           liftMotor.set(UP_SPEED);
            currentState = StateEnum.MiddleToTopLeavingMiddle ;
         } else {
           liftMotor.set(STALL_SPEED) ;
         }
         break ;
      case MiddleToTopLeavingMiddle:
         // if we've left the middle
         if(commandedState != CommandEnum.Top){
           liftMotor.set(DOWN_SPEED);
           currentState = StateEnum.TopToMiddle;
         }
         else if ( getLocation() == CommandEnum.None) {
             currentState = StateEnum.MiddleToTopWaitingTop ;
         }
         break ;
      case MiddleToTopWaitingTop:
      if(commandedState != CommandEnum.Top){
        liftMotor.set(DOWN_SPEED);
        currentState = StateEnum.TopToMiddle;
      }
     if(getLocation() == CommandEnum.Top){
           currentState = StateEnum.Top;
      }
         break;
      case MiddleToBottomleavingMiddle:
         if(commandedState != CommandEnum.Bottom){
           liftMotor.set(UP_SPEED);
           currentState = StateEnum.BottomToMiddle;
         }
         else if ( getLocation() == CommandEnum.None) {
            currentState = StateEnum.MiddleToBottomWaitingBottom ;
         }
         break ;
      case MiddleToBottomWaitingBottom:
        if(commandedState != CommandEnum.Bottom){
           liftMotor.set(UP_SPEED);
           currentState = StateEnum.BottomToMiddle;
         }
         else if ( getLocation() == CommandEnum.Bottom) {
           currentState = StateEnum.Bottom ;
         } 
         break;

      case Top:
         if(commandedState != CommandEnum.Top){
           liftMotor.set(DOWN_SPEED);
           currentState = StateEnum.TopToMiddle ;
         }
         else{
           liftMotor.set(STALL_SPEED);
         }
         
        break ;
       case TopToMiddle:
       if(commandedState == CommandEnum.Top){
         liftMotor.set(UP_SPEED);
         currentState = StateEnum.MiddleToTopLeavingMiddle;
       }
         else if( getLocation() == CommandEnum.Middle) {
           currentState = StateEnum.Middle ;
         }
         break ;
        }
      } 
    }else{
      liftMotor.set(DOWN_SPEED);
    } 
  }
  public CommandEnum getLocation(){
    if(!HES_F.get() && HES_B.get()){
      return CommandEnum.Bottom;
    }
    else if(!HES_F.get() && !HES_B.get()){
      return CommandEnum.Middle;
    }
    else if(HES_F.get() && !HES_B.get()){
      return CommandEnum.Top;
    }
    else{
      return CommandEnum.None;
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
