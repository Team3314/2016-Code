package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum autoCrossCenterDefenseShootHighStates {
	START,
	MOVEINTAKE,
	RESET,
	DRIVE,
	STOP,
	CENTER1,
	ADJUSTDISTANCE,
	CENTER2,
	SHOOT,
	DONE
}

public class AutoCrossCenterShootHigh implements Autonomous {
	
	autoCrossCenterDefenseShootHighStates currentState;
	autoCrossCenterDefenseShootHighStates nextState;
	Robot robot;
	double tolerance = 5;
	double distance;
	double desiredDistance;
	double time = 0;
	double firstAngle;
	double distanceBack;

	public AutoCrossCenterShootHigh(Robot myRobot) {
		robot = myRobot;
		currentState = autoCrossCenterDefenseShootHighStates.START;
		
	}
	
	public void reset() {
		currentState = autoCrossCenterDefenseShootHighStates.START;
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
			nextState = autoCrossCenterDefenseShootHighStates.MOVEINTAKE;
			break;
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = autoCrossCenterDefenseShootHighStates.DRIVE;
			}
			break;
		/*case RESET:
			if (time <= 0 && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = autoCrossCenterDefenseShootHighStates.DRIVE;
			}
			break;*/
		case DRIVE:
			if (distance >= desiredDistance ){//desiredDistance - distance <= tolerance) {
				nextState = autoCrossCenterDefenseShootHighStates.STOP;
			}
			break;
		case STOP:
			if (time <= 0 ){
				nextState = autoCrossCenterDefenseShootHighStates.CENTER1;
			}
			break;
		case CENTER1:
			if (robot.camera.centerX > 363  && robot.camera.centerX < 383){
				nextState = autoCrossCenterDefenseShootHighStates.ADJUSTDISTANCE;
			}
			break;
		case ADJUSTDISTANCE:
			if (robot.camera.centerY > 145 && robot.camera.centerY < 175)
				nextState = autoCrossCenterDefenseShootHighStates.CENTER2;
			break;
		case CENTER2:
			if (robot.camera.centerX > 363  && robot.camera.centerX < 383){
				nextState = autoCrossCenterDefenseShootHighStates.SHOOT;
			}
			break;
		case SHOOT:
			if (time <= 0 ){
				nextState = autoCrossCenterDefenseShootHighStates.DONE;
			}
			break;
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == autoCrossCenterDefenseShootHighStates.START && nextState == autoCrossCenterDefenseShootHighStates.MOVEINTAKE) {
		
			robot.hal.intake.set(Value.kForward);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 25;
		}
		
		/*if (currentState == autoCrossCenterDefenseShootHighStates.MOVEINTAKE && nextState == autoCrossCenterDefenseShootHighStates.RESET) {
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 50;
		}*/
		
		if (currentState == autoCrossCenterDefenseShootHighStates.MOVEINTAKE && nextState == autoCrossCenterDefenseShootHighStates.DRIVE) {
			desiredDistance = 2.544;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-.75);
		}
		
		if (currentState == autoCrossCenterDefenseShootHighStates.DRIVE && nextState == autoCrossCenterDefenseShootHighStates.STOP) {
			robot.cockRequest = true;
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.hal.intake.set(Value.kReverse);
			time = 50;
			robot.tankDrive.setDriveMode(driveMode.TANK);
		}
		
		if (currentState == autoCrossCenterDefenseShootHighStates.STOP && nextState == autoCrossCenterDefenseShootHighStates.CENTER1) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.cockRequest = false;
			robot.camera.centerCamera();
		}
		
		if (currentState == autoCrossCenterDefenseShootHighStates.CENTER1 && nextState == autoCrossCenterDefenseShootHighStates.ADJUSTDISTANCE) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.camera.adjustDistance();
		}
		
		if (currentState == autoCrossCenterDefenseShootHighStates.ADJUSTDISTANCE && nextState == autoCrossCenterDefenseShootHighStates.CENTER2) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.camera.centerCamera();
		}
		
		if (currentState == autoCrossCenterDefenseShootHighStates.CENTER2 && nextState == autoCrossCenterDefenseShootHighStates.SHOOT) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.shootRequest = true;
			time = 50;
		}
		
		if (currentState == autoCrossCenterDefenseShootHighStates.SHOOT && nextState == autoCrossCenterDefenseShootHighStates.DONE) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.shootRequest = false;
		}
	}
}
