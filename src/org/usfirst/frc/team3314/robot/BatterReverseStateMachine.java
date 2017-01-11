package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum batterReverseStates {
	START,
	READY,
	PAUSE,
	REVERSE,
	STOP,
	DONE
}

public class BatterReverseStateMachine {
	
	batterReverseStates currentState;
	batterReverseStates nextState;
	Robot robot;
	double tolerance = 1;
	double distance;
	double desiredDistance;
	double time = 50;
	double distanceError;
	double lastDistanceError;
	double batterReversePConstant = -2;
	double batterReverseDConstant = -20;
	double correction;

	public BatterReverseStateMachine(Robot myRobot, batterReverseStates firstState) {
		robot = myRobot;
		currentState = firstState;
		
	}
	
	public void reset() {
		currentState = batterReverseStates.START;
	}
	
	
	public void updateState() {
		distance = (robot.tankDrive.rightDriveTalon1.getPosition() - robot.tankDrive.leftDriveTalon1.getPosition()) / 2;
		distanceError = desiredDistance - distance;
		
		calcNext();
		doTransition();
		
		robot.tankDrive.update();
		
		currentState = nextState;
		time --;
		
		SmartDashboard.putString("Auto State", currentState.toString());
		lastDistanceError = distanceError;
	}
	
	public void calcNext() {
		nextState = currentState;
		
		switch (currentState) {
		case START:
			if (robot.batterReverseRequest){
				nextState = batterReverseStates.READY;
			}
			break;
		case READY:
			if (time <= 0){
				nextState = batterReverseStates.PAUSE;
			}
			break;
		case PAUSE:
			if (time <= 0){
				nextState = batterReverseStates.REVERSE;
			}
			break;
		case REVERSE:
			if (distance < desiredDistance + .01) {
				nextState = batterReverseStates.STOP;
			}
			break;
		case STOP:
			if (!robot.batterReverseRequest){
				nextState = batterReverseStates.DONE;
			}
			break;
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == batterReverseStates.START && nextState == batterReverseStates.READY) {			
			robot.hal.gyro.reset();
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.hal.catapultAdjuster.set(Value.kForward);
			time = 25;
		}
		
		if (currentState == batterReverseStates.READY && nextState == batterReverseStates.PAUSE) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 25;
			
		}
		
		if (currentState == batterReverseStates.PAUSE && nextState == batterReverseStates.REVERSE) {
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(correction);
			desiredDistance = -0.148;
			time = 25;
			
		}
		
		if (currentState == batterReverseStates.REVERSE && nextState == batterReverseStates.STOP) {
			robot.tankDrive.setDriveTrainSpeed(0);
		}
		
	}
	
	public void doState(){
		if (currentState == batterReverseStates.REVERSE){
			correction = distanceError * batterReversePConstant;
			correction += (distanceError - lastDistanceError) * batterReverseDConstant;
			if (correction > .25){
				correction = .25;
			}
			
			if (correction < -.25){
				correction = -.25;
			}
		}
	}
}
