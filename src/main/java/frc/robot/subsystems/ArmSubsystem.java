/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.ArmCommand;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmSubsystem extends Subsystem {
  public static Compressor compressor = new Compressor(); 
	public static Solenoid openSolenoidClaw = new Solenoid(RobotMap.openClawSolenoidPort);
	public static Solenoid closeSolenoidClaw = new Solenoid(RobotMap.closeClawSolenoidPort);
  public static Solenoid openSolenoidExtend = new Solenoid(RobotMap.openExtendSolenoidPort);
	public static Solenoid closeSolenoidExtend = new Solenoid(RobotMap.closeExtendSolenoidPort);
  public static boolean openClaw = false;
  public static boolean openExtend = false;
  public boolean hasDisk = false;

    public void initDefaultCommand() {
      setDefaultCommand(new ArmCommand());	
      
    }
    
    public boolean getClawStatus() {
    	return openSolenoidClaw.get();
    }
    /**
     * sets the open variable to true and updates the solenoids
     * */
    public void openClaw() {
      openClaw = true;
      updateClawSolenoids();

    }
    
    public void toggleClaw() {
    	openClaw = !openClaw;
      updateClawSolenoids();
      SmartDashboard.putBoolean("Claw Open", openClaw);

    }
    
    /**
     * sets the open variable to false and updates the solenoids
     * */
    public void closeClaw() {
    	openClaw = false;
		updateClawSolenoids();
    }
    /**
     * sets the solenoids to be opposite states of each other using
     * the "open" variable
     * */
    private void updateClawSolenoids() {
    	openSolenoidClaw.set(openClaw);
    	closeSolenoidClaw.set(!openClaw);
    }




    public boolean getExtendStatus() {
    	return openSolenoidExtend.get();
    }
    /**
     * sets the open variable to true and updates the solenoids
     * */
    public void openExtend() {
      openExtend = true;
		  updateExtendSolenoids();
    }
    
    public void toggleExtend() {
    	openExtend = !openExtend;
      updateExtendSolenoids();
      SmartDashboard.putBoolean("Arm Extended", openExtend);
    }
    
    /**
     * sets the open variable to false and updates the solenoids
     * */
    public void closeExtend() {
    	openExtend = false;
		  updateExtendSolenoids();
    }
    /**
     * sets the solenoids to be opposite states of each other using
     * the "open" variable
     * */
    private void updateExtendSolenoids() {
    	openSolenoidExtend.set(openExtend);
    	closeSolenoidExtend.set(!openExtend);
    }

    public void getDisk(){
      
        openExtend();
        openClaw();
        hasDisk = true;
    
    }

    public void putDisk(){
        closeExtend();
        closeClaw();
        hasDisk = false;

    }

}
