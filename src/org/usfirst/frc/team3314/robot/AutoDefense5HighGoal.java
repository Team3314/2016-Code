package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum autoDefense5HighStates {
	START,
	MOVEINTAKE,
	RESET,
	DRIVE,
	STOP,
	CENTER1,
	ADJUSTDISTANCE,
	CENTER2,
	SHOOT,
	ANGLE,
	STOP2,
	REVERSE1,
	STOP3,
	ANGLE2,
	STOP4,
	REVERSE2,
	DONE
}

public class AutoDefense5HighGoal implements Autonomous {
	
	autoDefense5HighStates currentState;
	autoDefense5HighStates nextState;
	Robot robot;
	double tolerance = 5;
	double distance;
	double desiredDistance;
	double time = 0;
	double firstAngle;
	double distanceBack;
	double secondAngle;
	double desiredAngle;

	public AutoDefense5HighGoal(Robot myRobot) {
		robot = myRobot;
		currentState = autoDefense5HighStates.START;
		
	}
	
	public void reset() {
		currentState = autoDefense5HighStates.START;
	}
	
	
	public void updateState() {
		distance = robot.tankDrive.getDistance();
		
		calcNext();
		doTransition();
		
		currentState = nextState;
		time --;
		
		SmartDashboard.putString("Auto State", currentState.toString());
		
	}
	
	public void calcNext() {
		nextState = currentState;
		
		switch (currentState) {
		case START:
			robot.hal.gyro.reset();
			nextState = autoDefense5HighStates.MOVEINTAKE;
			break;
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = autoDefense5HighStates.DRIVE;
			}
			break;
		/*case RESET:
			if (time <= 0 && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = autoDefense5HighStates.DRIVE;
			}
			break;*/
		case DRIVE:
			if (distance >= desiredDistance ){//desiredDistance - distance <= tolerance) {
				nextState = autoDefense5HighStates.STOP;
			}
			break;
		case STOP:
			if (time <= 0 ){
				nextState = autoDefense5HighStates.CENTER1;
			}
			break;
		case CENTER1:
			if (robot.camera.centerX > 363  && robot.camera.centerX < 383){
				firstAngle = robot.hal.gyro.angle();
				nextState = autoDefense5HighStates.ADJUSTDISTANCE;
			}
			break;
		case ADJUSTDISTANCE:
			if (robot.camera.centerY > 145 && robot.camera.centerY < 175)
				distanceBack = robot.tankDrive.getDistance();
				nextState = autoDefense5HighStates.CENTER2;
			break;
		case CENTER2:
			if (robot.camera.centerX > 363  && robot.camera.centerX < 383){
				secondAngle = robot.hal.gyro.angle();
				nextState = autoDefense5HighStates.SHOOT;
			}
			break;
		case SHOOT:
			if (time <= 0 ){
				nextState = autoDefense5HighStates.ANGLE;
			}
			break;
		case ANGLE:
			if (Math.abs(desiredAngle - robot.hal.gyro.angle()) < 1){
				nextState = autoDefense5HighStates.STOP2;
			}
			break;
		case STOP2:
			if (time <= 0 ){
				nextState = autoDefense5HighStates.REVERSE1;
			}
			break;
		case REVERSE1:
			if (robot.tankDrive.getDistance() <= -distanceBack){
				nextState = autoDefense5HighStates.STOP3;
			}
			break;
		case STOP3:
			if (time <= 0 ){
				nextState = autoDefense5HighStates.ANGLE2;
			}
			break;
		case ANGLE2:
			if (Math.abs(desiredAngle - robot.hal.gyro.angle()) < 1){
				nextState = autoDefense5HighStates.STOP4;
			}
			break;
		case STOP4:
			if (time <= 0 ){
				nextState = autoDefense5HighStates.REVERSE2;
			}
			break;
		case REVERSE2:
			if (distance <= desiredDistance){
				nextState = autoDefense5HighStates.DONE;
			}
			break;
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == autoDefense5HighStates.START && nextState == autoDefense5HighStates.MOVEINTAKE) {
		
			robot.hal.intake.set(Value.kForward);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 25;
		}
		
		/*if (currentState == autoDefense5HighStates.MOVEINTAKE && nextState == autoDefense5HighStates.RESET) {
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 50;
		}*/
		
		if (currentState == autoDefense5HighStates.MOVEINTAKE && nextState == autoDefense5HighStates.DRIVE) {
			desiredDistance = 2.544;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-.75);
		}
		
		if (currentState == autoDefense5HighStates.DRIVE && nextState == autoDefense5HighStates.STOP) {
			robot.cockRequest = true;
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.hal.intake.set(Value.kReverse);
			time = 50;
			robot.tankDrive.setDriveMode(driveMode.TANK);
		}
		
		if (currentState == autoDefense5HighStates.STOP && nextState == autoDefense5HighStates.CENTER1) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.cockRequest = false;
			robot.camera.centerCamera();
		}
		
		if (currentState == autoDefense5HighStates.CENTER1 && nextState == autoDefense5HighStates.ADJUSTDISTANCE) {
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.camera.adjustDistance();
		}
		
		if (currentState == autoDefense5HighStates.ADJUSTDISTANCE && nextState == autoDefense5HighStates.CENTER2) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.camera.centerCamera();
		}
		
		if (currentState == autoDefense5HighStates.CENTER2 && nextState == autoDefense5HighStates.SHOOT) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.shootRequest = true;
			robot.hal.gyro.reset();
			time = 50;
		}
		
		if (currentState == autoDefense5HighStates.SHOOT && nextState == autoDefense5HighStates.ANGLE) {
			robot.shootRequest = false;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(-firstAngle);
		}
		
		
		if (currentState == autoDefense5HighStates.ANGLE && nextState == autoDefense5HighStates.STOP2) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 25;
		}
		
		if (currentState == autoDefense5HighStates.STOP2 && nextState == autoDefense5HighStates.REVERSE1) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(.25);
			robot.tankDrive.setDriveAngle(robot.hal.gyro.angle());
		}
		
		if (currentState == autoDefense5HighStates.REVERSE1 && nextState == autoDefense5HighStates.STOP3) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			time = 25;
		}
		
		if (currentState == autoDefense5HighStates.STOP3 && nextState == autoDefense5HighStates.ANGLE2) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(-secondAngle);
		}
		
		if (currentState == autoDefense5HighStates.ANGLE2 && nextState == autoDefense5HighStates.STOP4) {
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			time = 25;
		}
		
		if (currentState == autoDefense5HighStates.STOP4 && nextState == autoDefense5HighStates.REVERSE2) {
			desiredDistance = 2.25;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(-.75);
			robot.tankDrive.setDriveAngle(robot.hal.gyro.angle());
			
		}
		
		if (currentState == autoDefense5HighStates.REVERSE2 && nextState == autoDefense5HighStates.DONE) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		}
	}
}
