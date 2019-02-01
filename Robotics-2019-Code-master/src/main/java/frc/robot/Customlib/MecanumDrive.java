// package frc.robot.Customlib;

// import java.util.Vector;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PWMSpeedController;
// import edu.wpi.first.wpilibj.drive.Vector2d;

// public class MecanumDrive{

//     private double[] speedMods = {.8,1,.8,.8};
//     private PWMSpeedController LFront,RFront,LBack,RBack;
//     private Joystick joyStick;
//     public class NonNormalizedNumber extends Exception{public NonNormalizedNumber(String message){super(message);}};

//     public MecanumDrive(PWMSpeedController LFront, PWMSpeedController RFront, PWMSpeedController LBack, PWMSpeedController RBack){
//         this.LFront = LFront;
//         this.RFront = RFront;
//         this.LBack = LBack;
//         this.RBack = RBack;
//     }

//     public void drive(){
//         drive(joyStick);
//     }
//     public void drive(Joystick stick){
//         try{
//             drive(stick.getX(),stick.getY(),stick.getZ());
//         }catch(NonNormalizedNumber e){} //This will never happen
//     }

//     public void setJoystick(Joystick joyStick){this.joyStick = joyStick;}

//     /**
//      * @param wheel wheel number: 0 = Front Left, 1 = Front Right, 2 = Back Left, 3 = Back Right
//      * @param speed speed modifier
//      */
//     public void setSpeedMod(int wheel, double speed) throws NonNormalizedNumber{
//         normalCheck(speed);
//         speedMods[wheel] = speed;
//     }

//     /**
//      * Negitive = Left, Postive = Right
//      */
//     public void drive(double xAxis, double yAxis, double zAxis) throws NonNormalizedNumber{
//         double[] temp = {xAxis,yAxis,zAxis};
//         normalCheck(temp);
//         xAxis *= -1;
//         double[] temp2 = rotateVector(xAxis, yAxis, 0);
//         xAxis = temp2[0];
//         yAxis = temp2[1];
//         //xAxis = Math.pow(xAxis,3);
//         //yAxis = Math.pow(yAxis,3);
//         double[] wheelSpeeds = new double[4];
//         wheelSpeeds[0] = (yAxis+xAxis-zAxis)*speedMods[0];
//         wheelSpeeds[1] = (yAxis-xAxis-zAxis)*speedMods[1];
//         wheelSpeeds[2] = (yAxis-xAxis+zAxis)*speedMods[2];
//         wheelSpeeds[3] = (yAxis+xAxis+zAxis)*speedMods[3];
        
        
//         normalize(wheelSpeeds);
//         LFront.set(wheelSpeeds[0]);
//         RFront.set(wheelSpeeds[1]);
//         LBack.set(wheelSpeeds[2]);
//         RBack.set(wheelSpeeds[3]);  //Set Back Right motor to backwards
//     }
    
//     public double getAngle(Vector2d a, Vector2d b){
//         double angle = Math.acos(a.dot(b)/(a.magnitude()*b.magnitude()));
//         return angle;
//     }

//     public void normalCheck(double[] nums) throws NonNormalizedNumber{
//         for(double num : nums)
//         if(Math.abs(num) > 1) throw new NonNormalizedNumber(num + " is greater than 1 or less than -1, use a normalized number");
//     }
//     public void normalCheck(double num) throws NonNormalizedNumber{
//         if(Math.abs(num) > 1) throw new NonNormalizedNumber(num + " is greater than 1 or less than -1, use a normalized number");
//     }
//     protected static double[] rotateVector(double x, double y, double angle) {
//         double cosA = Math.cos(angle * (Math.PI / 180.0));
//         double sinA = Math.sin(angle * (Math.PI / 180.0));
//         double[] out = new double[2];
//         out[0] = x * cosA - y * sinA;
//         out[1] = x * sinA + y * cosA;
//         return out;
//       }
//    /**
//    * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
//    */
//   private void normalize(double[] wheelSpeeds) {
//     double maxMagnitude = Math.abs(wheelSpeeds[0]);
//     for (int i = 1; i < wheelSpeeds.length; i++) {
//       double temp = Math.abs(wheelSpeeds[i]);
//       if (maxMagnitude < temp) {
//         maxMagnitude = temp;
//       }
//     }
//     if (maxMagnitude > 1.0) {
//       for (int i = 0; i < wheelSpeeds.length; i++) {
//         wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
//       }
//     }
//   }

// }