package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;




public class HardwareAbstractionLayer {
	
	Robot robot;
	
	CANTalon intakeTalon;
	CANTalon climberTalon;
	
	//Digital I/O section
	DigitalInput latchSensor;
    DigitalInput catapultResetSensor;
    DigitalInput autoSelect1;
    DigitalInput autoSelect2;
    DigitalInput autoSelect4;
	
	//Pneumatics
	Compressor pcm1;
	
	Compressor pcm2;
	DoubleSolenoid latch;
	DoubleSolenoid intake;
	DoubleSolenoid catapultReset;
	DoubleSolenoid pto;
	DoubleSolenoid driveShifter;
	DoubleSolenoid catapultAdjuster;
	DoubleSolenoid climber;
	
	//Analog section
	CustomGyro gyro;
	
	//Relay section
	Relay flashlight;
	
	AnalogInput intakeSensor;
	
	double driveEncoderScale;
	
	public HardwareAbstractionLayer(Robot r){
		
		robot = r;
		
		intakeTalon = new CANTalon(7);
		climberTalon = new CANTalon(8);
		
		//Digital I/O Init
		latchSensor = new DigitalInput(5);
		catapultResetSensor = new DigitalInput(6);
		autoSelect1 = new DigitalInput(7);
		autoSelect2 = new DigitalInput(8);
		autoSelect4 = new DigitalInput(9);
		
		
		//Pneumatics
		pcm1 = new Compressor(0);
		pcm2 = new Compressor(1);
		latch = new DoubleSolenoid(0, 4, 5);
		intake = new DoubleSolenoid(0, 6, 7);
		catapultReset = new DoubleSolenoid(0, 0, 1);
		catapultAdjuster = new DoubleSolenoid(0, 2, 3);
		climber = new DoubleSolenoid(1, 4, 5);
		pto = new DoubleSolenoid(1  , 2, 3);
		driveShifter = new DoubleSolenoid(1, 0, 1);

		
		pcm1.setClosedLoopControl(true);
		pcm2.setClosedLoopControl(true);
		
		
		//Analog Init
		gyro = new CustomGyro(0, 1, 0.007);
		gyro.calibrate();
		intakeSensor = new AnalogInput(2);
		
		//Relay Init
		flashlight = new Relay(0);
	
    	driveEncoderScale = 2048;
		
	}


	public void reset(){
		gyro.reset();
	}
	
	public void getStatus(){
		gyro.update();
	}
}


