package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum autoCrossDefenseStates {
	START,
	MOVEINTAKE,
	RESET,
	DRIVE,
	STOP,
	DONE
}

public class AutoCrossDefense implements Autonomous {
	
	autoCrossDefenseStates currentState;
	autoCrossDefenseStates nextState;
	Robot robot;
	double tolerance = 5;
	double distance;
	double desiredDistance;
	double time = 0;

	public AutoCrossDefense(Robot myRobot) {
		robot = myRobot;
		currentState = autoCrossDefenseStates.START;
		
	}
	
	public void reset() {
		currentState = autoCrossDefenseStates.START;
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
			nextState = autoCrossDefenseStates.MOVEINTAKE;
			break;
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = autoCrossDefenseStates.DRIVE;
			}
			break;
		/*case RESET:
			if (time <= 0 && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = autoCrossDefenseStates.DRIVE;
			}
			break;*/
		case DRIVE:
			if (time <= 0 ){//desiredDistance - distance <= tolerance) {
				nextState = autoCrossDefenseStates.STOP;
			}
			break;
		case STOP:
			if (time <= 0 ){
				nextState = autoCrossDefenseStates.DONE;
			}
			break;
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == autoCrossDefenseStates.START && nextState == autoCrossDefenseStates.MOVEINTAKE) {	
			robot.hal.intake.set(Value.kForward);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 25;
		}
		
		/*
		if (currentState == autoCrossDefenseStates.MOVEINTAKE && nextState == autoCrossDefenseStates.RESET) {
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 50;
		}*/
		
		if (currentState == autoCrossDefenseStates.MOVEINTAKE && nextState == autoCrossDefenseStates.DRIVE) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-.75);
			time = 200;
		}
		
		if (currentState == autoCrossDefenseStates.DRIVE && nextState == autoCrossDefenseStates.STOP) {
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.hal.intake.set(Value.kReverse);
			time = 50;
		}
	}
}
