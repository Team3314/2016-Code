package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum AutoStates {
	START,
	DRIVE,
	STOP1,
	MOVEINTAKE,
	RESET,
	ANGLE,
	STOP2,
	SHOOT,
	ANGLE2,
	STOP3,
	REVERSE1,
	STOP4,
	ANGLE3,
	STOP5,
	REVERSE2,
	STOP6,
	FORWARD,
	DONE
}

public class AutoSpyBotHighGoalReverseCrossLowBar implements Autonomous {
	
	AutoStates currentState;
	AutoStates nextState;
	Robot robot;
	double desiredAngle;
	double tolerance = .05;
	double distance;
	double desiredDistance;
	double time = 0;

	public AutoSpyBotHighGoalReverseCrossLowBar(Robot myRobot) {
		robot = myRobot;
		currentState = AutoStates.START;
		
	}
	
	public void reset() {
		currentState = AutoStates.START;
	}
	
	
	public void updateState() {
		distance = robot.tankDrive.getDistance();
		
		
		calcNext();
		doTransition();
		
		currentState = nextState;
		time --;
		
		SmartDashboard.putString("Auto State", currentState.toString());
		SmartDashboard.putNumber("distance", robot.tankDrive.getDistance());
		SmartDashboard.putNumber("GYRO", robot.hal.gyro.angle());
	}
	
	public void calcNext() {
		nextState = currentState;
		
		switch (currentState) {
		case START:
			robot.hal.gyro.reset();
			nextState = AutoStates.DRIVE;
			break;
		case DRIVE:
			if (distance >= desiredDistance) {
				nextState = AutoStates.STOP1;
			}
			break;
		case STOP1:
			if (time <= 0 ){
				nextState = AutoStates.MOVEINTAKE;
			}
			break;
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = AutoStates.RESET;
			}
			break;
		case RESET:
			if (time <= 0 && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = AutoStates.ANGLE;
			}
			break;
		case ANGLE:
			if (robot.hal.gyro.angle() >= desiredAngle){
				nextState = AutoStates.STOP2;
			}
			break;
		case STOP2:
			if (time <= 0 ){
				nextState = AutoStates.SHOOT;
			}
			break;
		case SHOOT:
			if (time <= 0){//robot.catapultController.getCurrentState() == catapultState.DONE){//desiredDistance - distance <= tolerance) {
				nextState = AutoStates.ANGLE2;
			}
			break;
		case ANGLE2:
			if (robot.hal.gyro.angle() >= 110) {
				nextState = AutoStates.STOP3;
			}
			break;
		case STOP3:
			if (time <= 0) {
				nextState = AutoStates.REVERSE1;
			}
			break;
		case REVERSE1:
			if (distance >= desiredDistance) {
				nextState = AutoStates.DONE; //nextState = AutoStates.STOP4; 
			}
			break;/*
		case STOP4:
			if (time <= 0) {
				nextState = AutoStates.ANGLE3;
			}
			break;
		case ANGLE3:
			if (robot.hal.gyro.angle() <= 110) {
				nextState = AutoStates.STOP5;
			}
			break;
		case STOP5:
			if (time <= 0) {
				nextState = AutoStates.REVERSE2;
			}
			break;*//*
		case REVERSE2:
			if (distance <= desiredDistance) {
				nextState = AutoStates.STOP6;
			}
			break;
		case STOP6:
			if (time <= 0) {
				nextState = AutoStates.FORWARD;
			}
			break;
		case FORWARD:
			if (distance >= desiredDistance) {
				nextState = AutoStates.DONE;
			}
			break;*/
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == AutoStates.START && nextState == AutoStates.DRIVE) {
			desiredDistance = 0.318;
			
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-.5);
			
			
		}
		
		if (currentState == AutoStates.DRIVE && nextState == AutoStates.STOP1) {
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 13;
		
			
		}
		
		if (currentState == AutoStates.STOP1 && nextState == AutoStates.MOVEINTAKE) {
			//robot.cockRequest = true;
			robot.hal.intake.set(Value.kForward);
			time = 13;
		}
		
		if (currentState == AutoStates.MOVEINTAKE && nextState == AutoStates.RESET) {
			robot.hal.gyro.reset();
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 25;
		}
		
		
		
		if (currentState == AutoStates.RESET && nextState == AutoStates.ANGLE) {
			robot.cockRequest = true;
			robot.tankDrive.setDriveAngle(-5);
		//	robot.tankDrive.setDriveTrainSpeed(0);
			time = 50;
		}
		
		if (currentState == AutoStates.ANGLE && nextState == AutoStates.STOP2) {
			robot.cockRequest = false;
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 13;
			
		}
		
		if (currentState == AutoStates.STOP2 && nextState == AutoStates.SHOOT) {
			robot.shootRequest = true;
			time = 100;
		}
		
		if (currentState == AutoStates.SHOOT && nextState == AutoStates.ANGLE2) {
			robot.tankDrive.setDriveTrainSpeed(0);
			
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(-.5, 0);
			
			robot.shootRequest = false;
			robot.hal.intake.set(Value.kReverse);
			
			time = 50;
			
		}
		if (currentState == AutoStates.ANGLE2 && nextState == AutoStates.STOP3) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(robot.hal.gyro.angle());
			
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			desiredDistance = 2.544;
			time = 13;
		}
		if (currentState == AutoStates.STOP3 && nextState == AutoStates.REVERSE1) {
			robot.tankDrive.setDriveTrainSpeed(.375);
		}
		if (currentState == AutoStates.REVERSE1 && nextState == AutoStates.STOP4) {
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 13;
		}
		if (currentState == AutoStates.STOP4 && nextState == AutoStates.ANGLE3) {
			robot.tankDrive.setDriveAngle(90);
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 50;
		}
		if (currentState == AutoStates.ANGLE3 && nextState == AutoStates.STOP5) {
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			desiredDistance = 3.18;//6.343;
			robot.hal.intake.set(Value.kForward);
			time = 13;
		}
		if (currentState == AutoStates.STOP5 && nextState == AutoStates.REVERSE2) {
			robot.tankDrive.setDriveTrainSpeed(.25);
		}
		if (currentState == AutoStates.REVERSE2 && nextState == AutoStates.STOP6) {
			robot.tankDrive.setDriveAngle(90);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			desiredDistance = -3.71;
			time = 13;
		}
		if (currentState == AutoStates.STOP6 && nextState == AutoStates.FORWARD) {
			robot.tankDrive.setDriveTrainSpeed(-.25);
			
		}
		if (currentState == AutoStates.FORWARD  && nextState == AutoStates.DONE) {
			robot.tankDrive.setDriveTrainSpeed(0);
		}
		
		if (currentState == AutoStates.REVERSE1  && nextState == AutoStates.DONE) {
			robot.tankDrive.setDriveTrainSpeed(0);
		}
		
		if (currentState == AutoStates.STOP5  && nextState == AutoStates.DONE) {
			robot.tankDrive.setDriveTrainSpeed(0);
		}
	}
}
