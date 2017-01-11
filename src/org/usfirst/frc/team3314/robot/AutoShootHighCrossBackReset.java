package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum autoShootHighCrossBackResetStates {
	START,
	MOVEINTAKE,
	DRIVE,
	STOP,
	CALCANGLE1,
	GYROTURN1,
	DWELL1,
	ADJUSTDISTANCE,
	CALCANGLE2,
	GYROTURN2,
	DWELL2,
	CHECKANGLE,
	SHOOT,
	ANGLE,
	STOP2,
	REVERSE1,
	STOP3,
	ANGLE2,
	STOP4,
	REVERSE2,
	FORWARD,
	DONE
}

public class AutoShootHighCrossBackReset implements Autonomous {
	
	autoShootHighCrossBackResetStates currentState;
	autoShootHighCrossBackResetStates nextState;
	Robot robot;
	double tolerance = 5;
	double distance;
	double desiredDistance;
	double time = 0;
	double firstAngle;
	double secondAngle;
	double distanceBack;
	double desiredAngle;
	double cameraError;
	double xAngleError;
	double angleAccum;

	public AutoShootHighCrossBackReset(Robot myRobot) {
		
		robot = myRobot;
		currentState = autoShootHighCrossBackResetStates.START;
		angleAccum = 0;
	}
	
	public void reset() {
		currentState = autoShootHighCrossBackResetStates.START;
	}
	
	
	public void updateState() {
		distance = robot.tankDrive.getDistance();
		
		
		
		robot.camera.DisplayData();
		
		cameraError = robot.camera.centerX - 351
				;
		
		calcNext();
		doTransition();
		
		currentState = nextState;
		time --;
		
		SmartDashboard.putString("Auto State", currentState.toString());
		SmartDashboard.putNumber("Distance back", distanceBack);
		SmartDashboard.putNumber("AUTO DISTANCE", distance);
		
	}
	
	public void calcNext() {
		nextState = currentState;
		
		switch (currentState) {
		case START:
			robot.hal.gyro.reset();
			nextState = autoShootHighCrossBackResetStates.MOVEINTAKE;
			break;
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = autoShootHighCrossBackResetStates.DRIVE;
			}
			break;
		/*case RESET:
			if (time <= 0 && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = autoShootHighCrossBackResetStates.DRIVE;
			}
			break;*/
		case DRIVE:
			if (distance >= desiredDistance ){//desiredDistance - distance <= tolerance) {
				nextState = autoShootHighCrossBackResetStates.STOP;
			}
			break;
		case STOP:
			if (time <= 0 ){
				nextState = autoShootHighCrossBackResetStates.CALCANGLE1;
			}
			break;
		case CALCANGLE1:
			// xAngleError = cameraError * (Width of view in Degrees / width of view in pixels)
			xAngleError = cameraError * .09375;
			firstAngle = xAngleError;
			robot.hal.gyro.reset();
			nextState = autoShootHighCrossBackResetStates.GYROTURN1;
			break;
		case GYROTURN1:
			if ((Math.abs(xAngleError - robot.hal.gyro.angle())) < Math.abs(4)){
				nextState = autoShootHighCrossBackResetStates.DWELL1;
			}
			break;
		case DWELL1:
			if (time <= 0){
				nextState = autoShootHighCrossBackResetStates.ADJUSTDISTANCE;
			}
			break;
		case ADJUSTDISTANCE:
			if (robot.camera.centerY > 155 && robot.camera.centerY < 195){ //Previous values: 175, 205
				distanceBack = robot.tankDrive.getDistance();
				nextState = autoShootHighCrossBackResetStates.CALCANGLE2;
			}
			break;
		case CALCANGLE2:
			xAngleError = cameraError * .09375;
			secondAngle = xAngleError;
			angleAccum += secondAngle;
			robot.hal.gyro.reset();
			nextState = autoShootHighCrossBackResetStates.GYROTURN2;
			break;
		case GYROTURN2:
			if ((xAngleError - robot.hal.gyro.angle()) < Math.abs(4)){
				nextState = autoShootHighCrossBackResetStates.DWELL2;
			}
			break;
		case DWELL2:
			if (time <= 0){	
				nextState = autoShootHighCrossBackResetStates.CHECKANGLE;
			}
			break;
		case CHECKANGLE:
			xAngleError = cameraError * .09375;
			if (Math.abs(xAngleError) <= 3){
				nextState = autoShootHighCrossBackResetStates.SHOOT;
			}
			else {
				nextState = autoShootHighCrossBackResetStates.CALCANGLE2;
			}
			break;
		case SHOOT:
			if (robot.hal.latch.get().toString() == "kReverse"){
				nextState = autoShootHighCrossBackResetStates.ANGLE;
			}
			break;
		case ANGLE:
			if (Math.abs(robot.tankDrive.desiredAngle - robot.hal.gyro.angle()) < 3){
				nextState = autoShootHighCrossBackResetStates.STOP2;
			}
			break;
		case STOP2:
			if (time <= 0 ){
				nextState = autoShootHighCrossBackResetStates.REVERSE1;
			}
			break;
		case REVERSE1:
			robot.hal.catapultReset.set(Value.kForward);
			if (robot.tankDrive.getDistance() < -distanceBack){
				nextState = autoShootHighCrossBackResetStates.STOP3;
			}
			break;
		case STOP3:
			if (time <= 0 ){
				nextState = autoShootHighCrossBackResetStates.ANGLE2;
			}
			break;
		case ANGLE2:
			if (Math.abs(robot.tankDrive.desiredAngle - robot.hal.gyro.angle()) < 3){
				nextState = autoShootHighCrossBackResetStates.STOP4;
			}
			break;
		case STOP4:
			if (time <= 0 ){
				nextState = autoShootHighCrossBackResetStates.REVERSE2;
			}
			break;
		case REVERSE2:
			if (distance <= desiredDistance){
				nextState = autoShootHighCrossBackResetStates.DONE;
			}
			break;
		/*case FORWARD:
			if (time <= 0){
				nextState = autoShootHighCrossBackResetStates.DONE;
			}*/
		case DONE:
			if (!robot.hal.latchSensor.get()) {
				robot.hal.latch.set(Value.kForward);
			}
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == autoShootHighCrossBackResetStates.START && nextState == autoShootHighCrossBackResetStates.MOVEINTAKE) {
		
			robot.hal.intake.set(Value.kForward);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 25;
		}
		
		/*if (currentState == autoShootHighCrossBackResetStates.MOVEINTAKE && nextState == autoShootHighCrossBackResetStates.RESET) {
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 50;
		}*/
		
		if (currentState == autoShootHighCrossBackResetStates.MOVEINTAKE && nextState == autoShootHighCrossBackResetStates.DRIVE) {
			desiredDistance = 2.5;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-.75);
		}
		
		if (currentState == autoShootHighCrossBackResetStates.DRIVE && nextState == autoShootHighCrossBackResetStates.STOP) {
			robot.cockRequest = true;
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.hal.intake.set(Value.kReverse);
			time = 50;
			robot.tankDrive.setDriveMode(driveMode.TANK);
		}
		
		if (currentState == autoShootHighCrossBackResetStates.STOP && nextState == autoShootHighCrossBackResetStates.CALCANGLE1) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		
		}
		
		if (currentState == autoShootHighCrossBackResetStates.CALCANGLE1 && nextState == autoShootHighCrossBackResetStates.GYROTURN1) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(xAngleError);
			
			
		}
		
		if (currentState == autoShootHighCrossBackResetStates.GYROTURN1 && nextState == autoShootHighCrossBackResetStates.DWELL1) {
			time = 25;
		}
		
		if (currentState == autoShootHighCrossBackResetStates.DWELL1 && nextState == autoShootHighCrossBackResetStates.ADJUSTDISTANCE) {
			time = 250;
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.camera.adjustDistance();
		}
		
		if (currentState == autoShootHighCrossBackResetStates.ADJUSTDISTANCE && nextState == autoShootHighCrossBackResetStates.CALCANGLE2) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		}
		
		if (currentState == autoShootHighCrossBackResetStates.CALCANGLE2 && nextState == autoShootHighCrossBackResetStates.GYROTURN2) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(xAngleError);
			
			
		}
		
		if (currentState == autoShootHighCrossBackResetStates.GYROTURN2 && nextState == autoShootHighCrossBackResetStates.DWELL2) {
			time = 25;
		}
		
		if (currentState == autoShootHighCrossBackResetStates.DWELL2 && nextState == autoShootHighCrossBackResetStates.CHECKANGLE) {
		}
		
		
		if (currentState == autoShootHighCrossBackResetStates.CHECKANGLE && nextState == autoShootHighCrossBackResetStates.SHOOT) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.hal.latch.set(Value.kReverse);
			time = 50;
		}
		
		if (currentState == autoShootHighCrossBackResetStates.SHOOT && nextState == autoShootHighCrossBackResetStates.ANGLE) {
			robot.shootRequest = false;
			robot.cockRequest = false;
			robot.catapultResetRequest = true;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(-angleAccum);
		}
		
		
		if (currentState == autoShootHighCrossBackResetStates.ANGLE && nextState == autoShootHighCrossBackResetStates.STOP2) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 25;
			
		}
		
		if (currentState == autoShootHighCrossBackResetStates.STOP2 && nextState == autoShootHighCrossBackResetStates.REVERSE1) {
			
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(.25);
			robot.tankDrive.setDriveAngle(robot.hal.gyro.angle());
		}
		
		if (currentState == autoShootHighCrossBackResetStates.REVERSE1 && nextState == autoShootHighCrossBackResetStates.STOP3) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setStickInputs(0, 0);
			time = 25;
		}
		
		if (currentState == autoShootHighCrossBackResetStates.STOP3 && nextState == autoShootHighCrossBackResetStates.ANGLE2) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(-firstAngle);
		}
		
		if (currentState == autoShootHighCrossBackResetStates.ANGLE2 && nextState == autoShootHighCrossBackResetStates.STOP4) {
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			time = 25;
		}
		
		if (currentState == autoShootHighCrossBackResetStates.STOP4 && nextState == autoShootHighCrossBackResetStates.REVERSE2) {
			desiredDistance = -2.366;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(.75);
			robot.tankDrive.setDriveAngle(robot.hal.gyro.angle());
			robot.hal.intake.set(Value.kForward);
			robot.hal.intakeTalon.set(-1);
			time = 50;
			
		}
		
		if (currentState == autoShootHighCrossBackResetStates.REVERSE2 && nextState == autoShootHighCrossBackResetStates.DONE) {
			robot.catapultResetRequest = false;
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.hal.intake.set(Value.kReverse);
			robot.hal.intakeTalon.set(0);
		}
	}
}
