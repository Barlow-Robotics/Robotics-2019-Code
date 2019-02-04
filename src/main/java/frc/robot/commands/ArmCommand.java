/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.*;
import frc.robot.Robot;
import frc.robot.OI;

public class ArmCommand extends Command {
  boolean check1 = false;
  boolean check2 = false;
  public ArmCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.armSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if(OI.getPlaystation().getRawButton(4)){
      if(check1){
        Robot.armSubsystem.openClaw();
        check1 = !check1;
      }else{
        Robot.armSubsystem.closeClaw();
        check1 = !check1;
    }
  }
  if(OI.getPlaystation().getRawButton(5)){
    if(check2){
      Robot.armSubsystem.openExtend();
      check2 = !check2;
    }else{
      Robot.armSubsystem.openExtend();
      check2 = !check2;
  }
}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
