
package org.usfirst.frc.team3314.robot;


import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

 enum driveMode {
	IDLE,
	SYMMETRIC,
	TANK,
	GYROLOCK,
	ENCODERLOCK,
	BUMP
}

public class TankDriveTrain {
	CANTalon leftDriveTalon1;
	CANTalon leftDriveTalon2;
	CANTalon leftDriveTalon3;
	
	CANTalon rightDriveTalon1;
	CANTalon rightDriveTalon2;
	CANTalon rightDriveTalon3;
	
	Robot robot;
	
	double driveEncoderScale;
	double count;
	double rawLeftSpeed;
	double rawRightSpeed;
	double leftStickInput;
	double rightStickInput;
	double desiredSpeed;
	double desiredTimeOut;
	double doneTime;
	double desiredAngle;
	double gyroPconstant = .05;
	double encoderPconstant = .05;
	int caseCounter = 0;
	boolean timeOutOccured;
	
	driveMode currentMode = driveMode.IDLE;
	
	public TankDriveTrain(Robot r){
	
		robot = r;
		leftDriveTalon1 = new CANTalon(1);
		leftDriveTalon2 = new CANTalon(3);
		leftDriveTalon3 = new CANTalon(5);
		
		rightDriveTalon1 = new CANTalon(2);
		rightDriveTalon2 = new CANTalon(4);
		rightDriveTalon3 = new CANTalon(6);
		
		leftDriveTalon2.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftDriveTalon2.set(leftDriveTalon1.getDeviceID());
		leftDriveTalon3.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftDriveTalon3.set(leftDriveTalon1.getDeviceID());
		
		leftDriveTalon1.reverseOutput(true);
		leftDriveTalon2.reverseOutput(false);
		leftDriveTalon3.reverseOutput(false);
		
		rightDriveTalon2.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightDriveTalon2.set(rightDriveTalon1.getDeviceID());
		rightDriveTalon3.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightDriveTalon3.set(rightDriveTalon1.getDeviceID());
		
		leftDriveTalon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		rightDriveTalon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		
		rightDriveTalon1.configEncoderCodesPerRev(8196);
		rightDriveTalon1.configNominalOutputVoltage(+0.0f, -0.0f);
		rightDriveTalon1.configPeakOutputVoltage(+12.0f, -12.0f);
		rightDriveTalon1.setProfile(0);
		rightDriveTalon1.setF(0);
		rightDriveTalon1.setP(.1);
		rightDriveTalon1.setI(0);
		rightDriveTalon1.setD(2);
		rightDriveTalon1.enable();
		rightDriveTalon1.enableControl();
		
		leftDriveTalon1.configEncoderCodesPerRev(8196);
		leftDriveTalon1.configNominalOutputVoltage(+0.0f, -0.0f);
		leftDriveTalon1.configPeakOutputVoltage(+12.0f, -12.0f);
		leftDriveTalon1.setProfile(0);
		leftDriveTalon1.setF(0);
		leftDriveTalon1.setP(.1);
		leftDriveTalon1.setI(0);
		leftDriveTalon1.setD(2);
		leftDriveTalon1.enable();
		leftDriveTalon1.enableControl();
		

		rightDriveTalon1.enableBrakeMode(true);
		rightDriveTalon2.enableBrakeMode(true);
		rightDriveTalon3.enableBrakeMode(true);
		
		leftDriveTalon1.enableBrakeMode(true);
		leftDriveTalon2.enableBrakeMode(true);
		leftDriveTalon3.enableBrakeMode(true);
		
		
		
		driveEncoderScale = 2048;
	
	}
	
	public void update() {
		switch (currentMode) {
		case IDLE:
			rawLeftSpeed = 0;
			rawRightSpeed = 0;
			break;
		case TANK:
			rawLeftSpeed = leftStickInput;
			rawRightSpeed = rightStickInput;
			break;
		case SYMMETRIC:
			if (robot.hi.leftTrackStick.getY() > 0) {
				rawLeftSpeed = 0; 
				rawRightSpeed = 0;
			}
			else {
			rawLeftSpeed = leftStickInput;
			rawRightSpeed = leftStickInput;
			}
			break;
		case BUMP:
			rawLeftSpeed = desiredSpeed;
			rawRightSpeed = desiredSpeed;
			break;
		case GYROLOCK:
			double currentAngle = robot.hal.gyro.angle();
			double errorAngle = desiredAngle - currentAngle;
			double correction;
			
			errorAngle = errorAngle % 360;
			
			while (errorAngle > 180){
				errorAngle -= 360;
			}
			
			while (errorAngle < -180){
				errorAngle += 360;
			}
			
			correction = errorAngle * gyroPconstant;
			
			SmartDashboard.putNumber("Error angle", errorAngle);
			SmartDashboard.putNumber("correction", correction);
			SmartDashboard.putNumber("desired angle", desiredAngle);
			SmartDashboard.putNumber("desired speed", desiredSpeed);
			
			rawLeftSpeed = desiredSpeed - (correction);
			rawRightSpeed = desiredSpeed + (correction);
			break;
		case ENCODERLOCK:
			double leftEncoderPosition = leftDriveTalon1.getPosition();
			double rightEncoderPosition = rightDriveTalon1.getPosition();
			double errorPosition = rightEncoderPosition - leftEncoderPosition;
			double encoderCorrection;
			
			
			encoderCorrection = errorPosition * encoderPconstant;
			
			rawLeftSpeed = desiredSpeed - (encoderCorrection);
			rawRightSpeed = desiredSpeed + (encoderCorrection);
			break;
		}
		
		if (robot.powerControlRequest){
			leftDriveTalon1.set(-rawLeftSpeed);
			rightDriveTalon1.set(rawRightSpeed);
		}
		else if (!robot.powerControlRequest && robot.hal.driveShifter.get().toString() == "kForward") {
			leftDriveTalon1.set(-rawLeftSpeed * 200);
			rightDriveTalon1.set(rawRightSpeed * 200);
		}
		else if (!robot.powerControlRequest && robot.hal.driveShifter.get().toString() == "kReverse") {
			leftDriveTalon1.set(-rawLeftSpeed * 75);
			rightDriveTalon1.set(rawRightSpeed * 75);
		}
		
		if (desiredTimeOut > 0 && System.currentTimeMillis() > doneTime){
			timeOutOccured = true;
		}
	}
	public void setDriveAngle(double angle) {
		desiredAngle = angle;
		
		
	}
	
	
	public void setStickInputs(double leftInput, double rightInput) {
	
		leftStickInput = leftInput;
		rightStickInput = rightInput;
	}
	
	public void setDriveMode(driveMode mode){
		currentMode = mode;
		
		switch(currentMode){
		case GYROLOCK:
			break;
		case ENCODERLOCK:
			leftDriveTalon1.setPosition(0);
			rightDriveTalon1.setPosition(0);
			break;
		}
	}
	
	
	public void setDriveTrainSpeed (double speed){	
		desiredSpeed = speed;
	}
	
	public void setDriveTimeOut (double time){
		timeOutOccured = false;
		desiredTimeOut = time;
		doneTime = System.currentTimeMillis() + (desiredTimeOut * 1000);
	}
	
	public double getDistance(){
		double distance;
		distance = (leftDriveTalon1.getPosition() - rightDriveTalon1.getPosition()) / 2;
		return distance;
	}
	
	
	public void getCurrent() {
		double leftTalon1Current = leftDriveTalon1.getOutputCurrent();
		double leftTalon2Current = leftDriveTalon2.getOutputCurrent();
		double leftTalon3Current = leftDriveTalon3.getOutputCurrent();
		
		double rightTalon1Current = rightDriveTalon1.getOutputCurrent();
		double rightTalon2Current = rightDriveTalon2.getOutputCurrent();
		double rightTalon3Current = rightDriveTalon2.getOutputCurrent();
		
		count++;
		if (count == 100) {
			if (leftTalon1Current > 0) {
				SmartDashboard.putNumber("Left Talon 1 Current", leftTalon1Current);
				
			}
			else {
				SmartDashboard.putString("LeftTalon1", "Left Talon 1 is not outputting power");
			}
			
			if (leftTalon2Current > 0) {
				SmartDashboard.putNumber("Left Talon 2 Current", leftTalon2Current);
				
			}
			else {
				SmartDashboard.putString("LeftTalon2", "Left Talon 2 is not outputting power");
			}
			
			if (leftTalon3Current > 0) {
				SmartDashboard.putNumber("Left Talon 3 Current", leftTalon3Current);
				
			}
			else {
				SmartDashboard.putString("LeftTalon3", "Left Talon 3 is not outputting power");
			}
			
			if (rightTalon1Current > 0) {
				SmartDashboard.putNumber("Right Talon 1 Current", rightTalon1Current);
				
			}
			else {
				SmartDashboard.putString("RightTalon1", "Right Talon 1 is not outputting power");
			}
			
			if (rightTalon2Current > 0) {
				SmartDashboard.putNumber("Right Talon 2 Current", rightTalon2Current);
				
			}
			else {
				SmartDashboard.putString("RightTalon2", "Right Talon 2 is not outputting power");
			}
			
			if (rightTalon3Current > 0) {
				SmartDashboard.putNumber("Right Talon 3 Current", rightTalon3Current);
				
			}
			else {
				SmartDashboard.putString("RightTalon3", "Right Talon 3 is not outputting power");
			}
		
		
		count = 0;
		}
		
		
		
	}
	
	public void reset(){
	
	}
	

}
