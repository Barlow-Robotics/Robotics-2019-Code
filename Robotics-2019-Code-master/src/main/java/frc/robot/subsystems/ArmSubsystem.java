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
import frc.robot.RobotMap;
import frc.robot.commands.ArmCommand;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

public class ArmSubsystem extends Subsystem {
    
	public Compressor compressor = new Compressor();
	public Solenoid openSolenoid = new Solenoid(RobotMap.openArmSolenoidPort);
	public Solenoid closeSolenoid = new Solenoid(RobotMap.closeArmSolenoidPort);
	
	public boolean open = true;

    public void initDefaultCommand() {
    	setDefaultCommand(new ArmCommand());	
    }
    
    public boolean getStatus() {
    	return openSolenoid.get();
    }
    /**
     * sets the open variable to true and updates the solenoids
     * */
    public void open() {
    	open = true;
		updateSolenoids();
    }
    
    public void toggle() {
    	open = !open;
    	updateSolenoids();
    }
    
    /**
     * sets the open variable to false and updates the solenoids
     * */
    public void close() {
    	open = false;
		updateSolenoids();
    }
    /**
     * sets the solenoids to be opposite states of each other using
     * the "open" variable
     * */
    private void updateSolenoids() {
    	openSolenoid.set(open);
    	closeSolenoid.set(!open);
    }
}
