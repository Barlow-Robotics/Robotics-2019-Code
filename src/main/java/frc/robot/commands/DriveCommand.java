package frc.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.Customlib.LimeLight;
import frc.robot.Customlib.VisionSystem;
import frc.robot.OI;
import frc.robot.subsystems.DriveSubsystem;

/**
 *
 */
public class DriveCommand extends Command {

    public DriveCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        
        // if(OI.getBox().getRawButtonPressed(10)){
        //     // Robot.driveSubsystem.visionSystem.limeLight.switchLED();
        // }
        
        Robot.driveSubsystem.doDriving();
        // System.out.println(DriveSubsystem.autoEnable);
        // DriveSubsystem.updateAngle();

        // if (DriveSubsystem.autoEnable)
        //     DriveSubsystem.focusTape();
        // else
        //     DriveSubsystem.mecanumDrive();

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
