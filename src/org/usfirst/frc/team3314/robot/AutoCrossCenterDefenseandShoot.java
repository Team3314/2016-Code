
package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum autoCrossCenterDefenseandShootStates {
	START,
	MOVEINTAKE,
	RESET,
	DRIVE,
	STOP,
	READY,
	REVERSE,
	DONE
}

public class AutoCrossCenterDefenseandShoot implements Autonomous {
	
	autoCrossCenterDefenseandShootStates currentState;
	autoCrossCenterDefenseandShootStates nextState;
	Robot robot;
	double tolerance = 5;
	double distance;
	double desiredDistance;
	double time = 0;

	public AutoCrossCenterDefenseandShoot(Robot myRobot) {
		robot = myRobot;
		currentState = autoCrossCenterDefenseandShootStates.START;
		
	}
	
	public void reset() {
		currentState = autoCrossCenterDefenseandShootStates.START;
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
			nextState = autoCrossCenterDefenseandShootStates.MOVEINTAKE;
			break;
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = autoCrossCenterDefenseandShootStates.RESET;
			}
			break;
		case RESET:
			if (time <= 0 && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = autoCrossCenterDefenseandShootStates.DRIVE;
			}
			break;
		case DRIVE:
			if (time <= 0 ){//desiredDistance - distance <= tolerance) {
				nextState = autoCrossCenterDefenseandShootStates.STOP;
			}
			break;
		case STOP:
			if (time <= 0 ){
				nextState = autoCrossCenterDefenseandShootStates.READY;
			}
			break;
		case READY:
			if (time <= 0){
				nextState = autoCrossCenterDefenseandShootStates.REVERSE;
			}
			break;
		case REVERSE:
			if (Math.abs(desiredDistance - distance) <= .01){
				nextState = autoCrossCenterDefenseandShootStates.DONE;
			}
			break;
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == autoCrossCenterDefenseandShootStates.START && nextState == autoCrossCenterDefenseandShootStates.MOVEINTAKE) {	
			robot.hal.intake.set(Value.kForward);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 25;
		}
		
		if (currentState == autoCrossCenterDefenseandShootStates.MOVEINTAKE && nextState == autoCrossCenterDefenseandShootStates.RESET) {
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 50;
		}
		
		if (currentState == autoCrossCenterDefenseandShootStates.RESET && nextState == autoCrossCenterDefenseandShootStates.DRIVE) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-.5);
			time = 500;
		}
		
		if (currentState == autoCrossCenterDefenseandShootStates.DRIVE && nextState == autoCrossCenterDefenseandShootStates.STOP) {
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 50;
		}
		
		if (currentState == autoCrossCenterDefenseandShootStates.STOP && nextState == autoCrossCenterDefenseandShootStates.READY) {
			robot.hal.gyro.reset();
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.hal.catapultAdjuster.set(Value.kForward);
			time = 25;
		}
		
		if (currentState == autoCrossCenterDefenseandShootStates.READY && nextState == autoCrossCenterDefenseandShootStates.REVERSE) {
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(.25);
			desiredDistance = -0.148;
		}
		
		if (currentState == autoCrossCenterDefenseandShootStates.REVERSE && nextState == autoCrossCenterDefenseandShootStates.DONE) {
			robot.tankDrive.setDriveTrainSpeed(0);
			time = 50;
		}
		
		
		
		
	}
}
