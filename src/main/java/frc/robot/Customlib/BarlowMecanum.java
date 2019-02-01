package frc.robot.Customlib;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SpeedController;

public class BarlowMecanum extends MecanumDrive {
    private static double multiplier = .9;

    public BarlowMecanum(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
    SpeedController frontRightMotor, SpeedController rearRightMotor){

        super(frontLeftMotor,frontRightMotor,rearLeftMotor,rearRightMotor);
    }

    @Override
    public void normalize(double[] wheelSpeeds) {

        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
          double temp = Math.abs(wheelSpeeds[i]);
          if (maxMagnitude < temp) {
            maxMagnitude = temp;
          }
        }
        if (maxMagnitude > 1.0) {
          for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude * multiplier;
          }
        }
    }

    public double getMultiplier(){
        return multiplier;
    }

    public void setMultiplier(double multi){
        multiplier = multi;
    }
}