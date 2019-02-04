package frc.robot.Customlib;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWM;

public class PIDSpeedCtrl implements SpeedController{

    // public double speed = 1;
    public boolean m_inverted = false;

    public SpeedController motorController;
    public PIDController pidCtrl ;
    public PIDSource pidSrc;

    // This should probably be updated to accept P, I, D, and F parameters for 
    // the PID
    public PIDSpeedCtrl(double Kp, double Ki, double Kd, double Kf, PIDSource pidSrc, SpeedController motorController ){
        this.motorController = motorController;
        this.pidSrc = pidSrc;
        pidSrc.setPIDSourceType(PIDSourceType.kRate);
        pidCtrl = new PIDController(Kp, Ki, Kd, Kf, pidSrc, motorController);
        
    }

    @Override
    public void set(double speed){
        System.out.println(((PWM)motorController).getName() + ": " + speed);
        pidCtrl.setSetpoint(speed);
    }

    @Override
    public double get(){
        return pidCtrl.getSetpoint() ;
    }

    @Override
    public void setInverted(boolean isInverted){
        motorController.setInverted(isInverted) ;
    }

    @Override
    public boolean getInverted(){
        return motorController.getInverted() ;
    }

    @Override
    public void disable() {
        motorController.disable() ;
        // do we need to do something with the PID? probably not.
      }

    @Override
    public void stopMotor(){
        motorController.stopMotor() ;

        // do we need to add some logic to disable the PID controller?

    }

    public void reset() {
        pidCtrl.reset() ;
    }

    @Override
    public void pidWrite(double output){
        // this member function is not implemented for because this class is not intended to be used as
        // a PIDOutput. It has its own embedded PIDController within.
        throw new UnsupportedOperationException("pidWrite called but not applicable for PID Speed Controller") ;
    }
}