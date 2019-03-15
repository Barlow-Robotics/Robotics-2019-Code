/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.HopperCommand;

/**
 * Add your docs here.
 */
public class HopperSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static Solenoid openSolenoid = new Solenoid(RobotMap.openHopperPort);
	public static Solenoid closeSolenoid = new Solenoid(RobotMap.closeHopperPort);

  boolean open = false;
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new HopperCommand());
  }

  public void open(){
    open = true;
    updateSolenoids();
  }

  public void close(){
    open = false;
    updateSolenoids();
  }

  private void updateSolenoids() {
    openSolenoid.set(open);
    closeSolenoid.set(!open);
  }

}
