package frc.robot.Customlib;

import edu.wpi.first.wpilibj.SerialPort;
public class Arduino{
	public static SerialPort ard;
	public static String buff = "0\n";
	public static String con = "";
	public Arduino(){
		tryConnect();
	}

	public static void tryConnect(){
		try{
			ard = new SerialPort(9600, SerialPort.Port.kUSB);
			ard.disableTermination();
		}catch(Exception e){}

	}

	public static String getData(){
		String test = ard.readString();
		String temp = "";
		boolean write = false;
		for(int i = test.length()-1; i >= 0; i--){
			if(test.charAt(i) == '\n' && !write){
				write = true;
				continue;
			}
			if(test.charAt(i) == '\n') break;
			if(write) temp = test.charAt(i) + temp;
		} 
		test = temp;	
		// System.out.println(test);
		if(test.equalsIgnoreCase("nack") || test.equalsIgnoreCase("0\n")){
			ard.close();
			tryConnect();
			test = buff;
		}
		buff = test;
		return test;
	}
}