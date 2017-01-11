package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum autoLowBarHighGoalStates {
	START,
	MOVEINTAKE,
	RESET,
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
	DONE
}

public class AutoLowBarHighGoal implements Autonomous {
	
	autoLowBarHighGoalStates currentState;
	autoLowBarHighGoalStates nextState;
	Robot robot;
	double tolerance = 5;
	double distance;
	double desiredDistance;
	double time = 0;
	double firstAngle;
	double distanceBack;
	double cameraError;
	double xAngleError;

	public AutoLowBarHighGoal(Robot myRobot) {
		
		robot = myRobot;
		currentState = autoLowBarHighGoalStates.START;
	}
	
	public void reset() {
		currentState = autoLowBarHighGoalStates.START;
	}
	
	
	public void updateState() {
		distance = robot.tankDrive.getDistance();
		
		robot.camera.DisplayData();
		
		cameraError = robot.camera.centerX - 373;
		
		calcNext();
		doTransition();
		
		currentState = nextState;
		time --;
		
		SmartDashboard.putString("Auto State", currentState.toString());
		SmartDashboard.putNumber("Distance 1", distance);
		
	}
	
	public void calcNext() {
		nextState = currentState;
		
		switch (currentState) {
		case START:
			robot.hal.gyro.reset();
			nextState = autoLowBarHighGoalStates.MOVEINTAKE;
			break;
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = autoLowBarHighGoalStates.DRIVE;
			}
			break;
		/*case RESET:
			if (time <= 0 && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = autoLowBarHighGoalStates.DRIVE;
			}
			break;*/
		case DRIVE:
			if (distance >= desiredDistance ){//desiredDistance - distance <= tolerance) {
				nextState = autoLowBarHighGoalStates.STOP;
			}
			break;
		case STOP:
			if (time <= 0 ){
				nextState = autoLowBarHighGoalStates.CALCANGLE1;
			}
			break;
		case CALCANGLE1:
			xAngleError = cameraError * .09375;
			robot.hal.gyro.reset();
			nextState = autoLowBarHighGoalStates.GYROTURN1;
			break;
		case GYROTURN1:
			if ((Math.abs(xAngleError - robot.hal.gyro.angle())) < 2){
				nextState = autoLowBarHighGoalStates.DWELL1;
			}
			break;
		case DWELL1:
			if (time <= 0){
				nextState = autoLowBarHighGoalStates.ADJUSTDISTANCE;
			}
			break;
		case ADJUSTDISTANCE:
			if (robot.camera.centerY > 145 && robot.camera.centerY < 175)
				nextState = autoLowBarHighGoalStates.CALCANGLE2;
			break;
		case CALCANGLE2:
			xAngleError = cameraError * .09375;
			robot.hal.gyro.reset();
			nextState = autoLowBarHighGoalStates.GYROTURN2;
			break;
		case GYROTURN2:
			if (((xAngleError - robot.hal.gyro.angle()) < Math.abs(3))){
				nextState = autoLowBarHighGoalStates.DWELL2;
			}
			break;
		case DWELL2:
			if (time <= 0){
				nextState = autoLowBarHighGoalStates.CHECKANGLE;
			}
			break;
		case CHECKANGLE:
			xAngleError = cameraError * .09375;
			if (Math.abs(xAngleError) <= 2){
				nextState = autoLowBarHighGoalStates.SHOOT;
			}
			else {
				nextState = autoLowBarHighGoalStates.CALCANGLE2;
			}
			break;
		case SHOOT:
			if (robot.hal.latch.get().toString() == "kReverse"){
				nextState = autoLowBarHighGoalStates.DONE;
			}
			break;
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == autoLowBarHighGoalStates.START && nextState == autoLowBarHighGoalStates.MOVEINTAKE) {
		
			robot.hal.intake.set(Value.kForward);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 25;
		}
		
		/*if (currentState == autoLowBarHighGoalStates.MOVEINTAKE && nextState == autoLowBarHighGoalStates.RESET) {
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 50;
		}*/
		
		if (currentState == autoLowBarHighGoalStates.MOVEINTAKE && nextState == autoLowBarHighGoalStates.DRIVE) {
			desiredDistance = 2.544;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-.75);
		}
		
		if (currentState == autoLowBarHighGoalStates.DRIVE && nextState == autoLowBarHighGoalStates.STOP) {
			robot.cockRequest = true;
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.hal.intake.set(Value.kReverse);
			time = 50;
			robot.tankDrive.setDriveMode(driveMode.TANK);
		}
		
		if (currentState == autoLowBarHighGoalStates.STOP && nextState == autoLowBarHighGoalStates.CALCANGLE1) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		
		}
		
		if (currentState == autoLowBarHighGoalStates.CALCANGLE1 && nextState == autoLowBarHighGoalStates.GYROTURN1) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(xAngleError);
			
			
		}
		
		if (currentState == autoLowBarHighGoalStates.GYROTURN1 && nextState == autoLowBarHighGoalStates.DWELL1) {
			time = 25;
		}
		
		if (currentState == autoLowBarHighGoalStates.DWELL1 && nextState == autoLowBarHighGoalStates.ADJUSTDISTANCE) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.camera.adjustDistance();
		}
		
		if (currentState == autoLowBarHighGoalStates.ADJUSTDISTANCE && nextState == autoLowBarHighGoalStates.CALCANGLE2) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		}
		
		if (currentState == autoLowBarHighGoalStates.CALCANGLE2 && nextState == autoLowBarHighGoalStates.GYROTURN2) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(xAngleError);
			
			
		}
		
		if (currentState == autoLowBarHighGoalStates.GYROTURN2 && nextState == autoLowBarHighGoalStates.DWELL2) {
			time = 25;
		}
		
		if (currentState == autoLowBarHighGoalStates.DWELL2 && nextState == autoLowBarHighGoalStates.CHECKANGLE) {
		}
		
		
		if (currentState == autoLowBarHighGoalStates.CHECKANGLE && nextState == autoLowBarHighGoalStates.SHOOT) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.hal.latch.set(Value.kReverse);
			time = 50;
		}
		
		if (currentState == autoLowBarHighGoalStates.SHOOT && nextState == autoLowBarHighGoalStates.DONE) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			
		}
	}
}
