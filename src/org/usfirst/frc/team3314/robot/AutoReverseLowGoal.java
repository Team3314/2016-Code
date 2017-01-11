package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum autoReverseLowGoalStates {
	START,
	READY,
	REVERSE,
	STOP,
	DONE
}

public class AutoReverseLowGoal implements Autonomous {
	
	autoReverseLowGoalStates currentState;
	autoReverseLowGoalStates nextState;
	Robot robot;
	double tolerance = .01;
	double distance;
	double desiredDistance;
	double time = 50;

	public AutoReverseLowGoal(Robot myRobot) {
		robot = myRobot;
		currentState = autoReverseLowGoalStates.START;
		
	}
	
	public void reset() {
		currentState = autoReverseLowGoalStates.START;
	}
	
	
	public void updateState() {
		distance = (robot.tankDrive.rightDriveTalon1.getPosition() - robot.tankDrive.leftDriveTalon1.getPosition()) / 2;
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
			if (time <= 0){
				nextState = autoReverseLowGoalStates.READY;
			}
			break;
		case READY:
			if (time <= 0){
				nextState = autoReverseLowGoalStates.REVERSE;
			}
			break;
		case REVERSE:
			if (Math.abs(desiredDistance - distance) <= .01) {
				nextState = autoReverseLowGoalStates.STOP;
			}
			break;
		case STOP:
			if (time <= 0){
				nextState = autoReverseLowGoalStates.DONE;
			}
			break;
		case DONE:
			
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == autoReverseLowGoalStates.START && nextState == autoReverseLowGoalStates.READY) {			
			robot.hal.gyro.reset();
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			time = 25;
		}
		
		if (currentState == autoReverseLowGoalStates.READY && nextState == autoReverseLowGoalStates.REVERSE) {
			robot.hal.intake.set(Value.kReverse);
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(.25);
			desiredDistance = -2.008;
			time = 75;
			
		}
		
		if (currentState == autoReverseLowGoalStates.REVERSE && nextState == autoReverseLowGoalStates.STOP) {
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.hal.intakeTalon.set(1);
			
		}
		
		if (currentState == autoReverseLowGoalStates.STOP && nextState == autoReverseLowGoalStates.DONE) {
			robot.hal.intakeTalon.set(0);
			
		}
		
		
		
		
	}
}
