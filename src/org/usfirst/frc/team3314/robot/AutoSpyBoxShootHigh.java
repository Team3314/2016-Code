package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum AutoSpyBoxShootHighStates {
	START,
	DRIVE,
	STOP1,
	MOVEINTAKE,
	RESET,
	ANGLE,
	STOP2,
	SHOOT,
	DONE
}

public class AutoSpyBoxShootHigh {
	
	AutoSpyBoxShootHighStates currentState;
	AutoSpyBoxShootHighStates nextState;
	Robot robot;
	double desiredAngle;
	double tolerance = .05;
	double distance;
	double desiredDistance;
	double time = 0;

	public AutoSpyBoxShootHigh(Robot myRobot) {
		robot = myRobot;
		currentState = AutoSpyBoxShootHighStates.START;
		
	}
	
	public void reset() {
		currentState = AutoSpyBoxShootHighStates.START;
	}
	
	
	public void updateState() {
		distance = robot.tankDrive.getDistance();
		
		
		calcNext();
		doTransition();
		
		currentState = nextState;
		time --;
		
		SmartDashboard.putString("Auto State", currentState.toString());
		SmartDashboard.putNumber("distance", robot.tankDrive.getDistance());
	}
	
	public void calcNext() {
		nextState = currentState;
		
		switch (currentState) {
		case START:
			robot.hal.gyro.reset();
			nextState = AutoSpyBoxShootHighStates.DRIVE;
			break;
		case DRIVE:
			if (distance >= desiredDistance) {
				nextState = AutoSpyBoxShootHighStates.STOP1;
			}
			break;
		case STOP1:
			if (time <= 0 ){
				nextState = AutoSpyBoxShootHighStates.MOVEINTAKE;
			}
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = AutoSpyBoxShootHighStates.ANGLE;
			}
			break;
			/*
		case RESET:
			if (time <= 0  && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = AutoSpyBoxShootHighStates.ANGLE;
			}
			break;*/
		case ANGLE:
			if (robot.hal.gyro.angle() <= desiredAngle){
				nextState = AutoSpyBoxShootHighStates.STOP2;
			}
			break;
		case STOP2:
			if (time <= 0 ){
				nextState = AutoSpyBoxShootHighStates.SHOOT;
			}
			break;
		case SHOOT:
			if (robot.catapultController.getCurrentState() == catapultState.DONE){//desiredDistance - distance <= tolerance) {
				nextState = AutoSpyBoxShootHighStates.DONE;
			}
			break;
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == AutoSpyBoxShootHighStates.START && nextState == AutoSpyBoxShootHighStates.DRIVE) {
			desiredDistance = .318;
			
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-.5);
			
			
		}
		
		if (currentState == AutoSpyBoxShootHighStates.DRIVE && nextState == AutoSpyBoxShootHighStates.STOP1) {
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 50;
			
			
		}
		
		if (currentState == AutoSpyBoxShootHighStates.STOP1 && nextState == AutoSpyBoxShootHighStates.MOVEINTAKE) {
			//robot.cockRequest = true;
			robot.hal.intake.set(Value.kForward);
			time = 75;
		}
		/*
		if (currentState == AutoSpyBoxShootHighStates.MOVEINTAKE && nextState == AutoSpyBoxShootHighStates.RESET) {
			robot.hal.gyro.reset();
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 50;
		}
		*/
		if (currentState == AutoSpyBoxShootHighStates.MOVEINTAKE && nextState == AutoSpyBoxShootHighStates.ANGLE) {
			robot.hal.gyro.reset();
			robot.cockRequest = true;
			robot.tankDrive.setDriveAngle(-4);
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 75;
		}
		
		/*
		if (currentState == AutoSpyBoxShootHighStates.RESET && nextState == AutoSpyBoxShootHighStates.ANGLE) {
			robot.cockRequest = true;
				robot.tankDrive.setDriveAngle(-5);
				robot.tankDrive.setDriveTrainSpeed(0);
			time = 50;
		}*/
		
		if (currentState == AutoSpyBoxShootHighStates.ANGLE && nextState == AutoSpyBoxShootHighStates.STOP2) {
			robot.cockRequest = false;
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 25;
			
		}
		
		if (currentState == AutoSpyBoxShootHighStates.STOP2 && nextState == AutoSpyBoxShootHighStates.SHOOT) {
			robot.shootRequest = true;
		}
		
		if (currentState == AutoSpyBoxShootHighStates.SHOOT && nextState == AutoSpyBoxShootHighStates.DONE) {
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.shootRequest = false;
			robot.hal.intake.set(Value.kReverse);
			
		}
	}
}

