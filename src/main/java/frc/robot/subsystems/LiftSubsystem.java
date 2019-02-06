/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static int posistion = 0;
  public static boolean movinUp = false;
  enum Position{
    bottom(-1), middle(0), top(1), none(0);
    private int val;

    private Position(){}

    private Position(int value){
        val = value;
    }

    public int getValue(){
        return val;
    }

  }
  public static Spark LiftMotor = new Spark(RobotMap.PWM.LIFT_MOTOR);
  public static Position Lastposition = Position.bottom;
  public static Position goTo = Position.none;

  //public static DigitalInput HES_L = new DigitalInput(RobotMap.DIO.HES_M);
  //public static DigitalInput HES_R = new DigitalInput(RobotMap.DIO.HES_T);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static void setPosition(Position where){
      goTo = where;
  }

  private final int DELAY = 2;
  private int currentCnt = DELAY;
  public void liftIt(){
    // if(!HES_L.get() || !HES_R.get()){
    //   currentCnt--;
    // }else{
    //   currentCnt = DELAY;
    // }

    if(currentCnt == 0){
      checkPositions();
      currentCnt = DELAY;
    }

    /*
     * Set the position you want to go to 
     */
    if(OI.getBox().getRawButton(6)) goTo = Position.bottom; // Bottom
    if(OI.getBox().getRawButton(4)) goTo = Position.middle; // Middle
    if(OI.getBox().getRawButton(3)) goTo = Position.top;    // Top

    if(Lastposition == goTo) goTo = Position.none; //Go nowhere if you reach the destination 
    if(goTo == Position.middle){
      LiftMotor.set(-Lastposition.getValue() * directionModifier(-Lastposition.getValue())); //If middle set, move away from the top or bottom (Depends on last position)
    }else{
      LiftMotor.set(goTo.getValue() * directionModifier(goTo.getValue())); //Go to 
    }

  }

  public static final double DWN_SPEED = 0; 
  public double directionModifier(double input){
    return input > 0 ? 0.8 : 0.5;
  }
    /*
     * Set the last position to the last sensor to detect the lift 
     */
  public void checkPositions(){
    // if(!HES_L.get() && HES_R.get()) Lastposition = Position.bottom;
    // if(!HES_L.get() && !HES_R.get()) Lastposition = Lastposition.getValue() == 1 ? Position.bottom : Position.top;
    // if(HES_L.get() && !HES_R.get()) Lastposition = Position.top;
  }
}
