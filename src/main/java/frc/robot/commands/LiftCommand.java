/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.CommandEnum;

public class LiftCommand extends Command {


  public LiftCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.liftSubsystem);
  }
  boolean startLift = true;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putBoolean("HES_B", Robot.liftSubsystem.HES_B.get());
    SmartDashboard.putBoolean("HES_F", Robot.liftSubsystem.HES_F.get());

    if(startLift && (Robot.liftSubsystem.getLocation() == CommandEnum.Bottom ||
           Robot.liftSubsystem.getLocation() == CommandEnum.Middle)){
             Robot.liftSubsystem.commandedState = CommandEnum.Bottom;
             startLift = false;
           }
    Robot.liftSubsystem.lift();
    if(OI.getBox().getRawButton(6)) Robot.liftSubsystem.commandedState = LiftSubsystem.CommandEnum.Bottom;
    if(OI.getBox().getRawButton(4)) Robot.liftSubsystem.commandedState = LiftSubsystem.CommandEnum.Middle;
    if(OI.getBox().getRawButton(3)) Robot.liftSubsystem.commandedState = LiftSubsystem.CommandEnum.Top;
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
