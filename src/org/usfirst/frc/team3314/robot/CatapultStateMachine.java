package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum catapultState {
	START,
	COCK,
	COCKED,
	CHECKCOCK,
	SHOOT,
	DUMP,
	UNLATCH,
	UNCOCK,
	BRINGDOWN,
	LATCH,
	DONE
}

public class CatapultStateMachine {
	final double TOLERANCE = .5;
	catapultState currentState;
	catapultState nextState;
	Robot robot;
	double angle = 0;
	double time;
	double desiredAngle;
	double currentAngle;
	boolean isCocked;

	public CatapultStateMachine(Robot myRobot, catapultState firstState) {
		robot = myRobot;
		currentState = firstState;
		isCocked = false;
	}
	
	public catapultState getCurrentState(){
		return currentState;
	}
	
	public void updateState() {
		calcNext();
		//doTransition();
		doNextState();
		
		currentState = nextState;
	}
	
	public void calcNext() {
		
		nextState = currentState;
		switch (currentState) {
		case START:
			
			if (robot.shootRequest) {
				time = 150;
				nextState = catapultState.CHECKCOCK;
			}
			if (robot.dumpRequest) {
				
				nextState = catapultState.UNCOCK;
			}
			if (robot.cockRequest) { //&& robot.hal.latchSensor.get()) {
				nextState = catapultState.COCK;
			}
			
			break;
			
		case COCKED:
			
			if (robot.shootRequest) {
				time = 150;
				nextState = catapultState.CHECKCOCK;
			}
			if (robot.dumpRequest) {
				
				nextState = catapultState.UNCOCK;
			}
			if (robot.cockRequest) { //&& robot.hal.latchSensor.get()) {
				nextState = catapultState.COCK;
			}
			break;
			
		case COCK:
			robot.hal.catapultReset.set(Value.kReverse);
			//robot.hal.intake.set(Value.kForward);
			nextState = catapultState.COCKED;
			break;
		case CHECKCOCK:
		/*	if (robot.hal.catapultReset.get().toString() == "kReverse" && time == 150) {
				nextState = catapultState.SHOOT;
			}*/
			//robot.hal.intake.set(Value.kForward);
			time--;
			if (robot.hal.catapultReset.get().toString() == "kForward" || robot.hal.catapultReset.get().toString() == "kOff") {
				robot.hal.catapultReset.set(Value.kReverse);
				
			}
			if (time <= 0 || !robot.hal.catapultResetSensor.get()) {
				time = 50;
				nextState = catapultState.SHOOT;
			}
				
			break;
		case SHOOT:
			robot.hal.latch.set(Value.kReverse);
			time--;
			if (time <= 0) {
				time  = 50;
				nextState = catapultState.BRINGDOWN;
			}
				
			break;
		case UNCOCK:
			if (robot.hal.catapultReset.get().toString() == "kReverse" || robot.hal.catapultReset.get().toString() == "kOff") {
				robot.hal.catapultReset.set(Value.kForward);
			}
			time--;
			if(time <= 0) {
				time = 25;
				nextState = catapultState.UNLATCH;
			}
			break;
		case UNLATCH:
			robot.hal.latch.set(Value.kReverse);
			time --;
			if (time <= 0){
				time = 100;
				nextState = catapultState.DUMP;
			}
			break;
		case DUMP:
			robot.hal.catapultReset.set(Value.kReverse);
			time --;
			if (time <= 0){
				time = 50;
				nextState = catapultState.BRINGDOWN;
			}
			break;
		case BRINGDOWN:
			time --;
			robot.hal.catapultAdjuster.set(Value.kReverse);
			robot.hal.catapultReset.set(Value.kForward);
			if (!robot.hal.latchSensor.get()) {
				
				time = 50;
				nextState = catapultState.LATCH;
			}
			break;
		case LATCH:
			robot.hal.latch.set(Value.kForward);
			time--;
			if (time <= 0) {
				nextState = catapultState.DONE;
			}
			break;
		case DONE:
			nextState = catapultState.START;
			break;
		}
	}
	/*
	public void doTransition() {
		if (nextState == catapultState.START && currentState != catapultState.START) {
			robot.hal.latch.set(Value.kForward);
			robot.hal.catapultReset.set(Value.kForward);
			time = 100;
		}
		
		
		if (currentState == catapultState.START && nextState == catapultState.SHOOT) {
			if (!isCocked){
				robot.hal.catapultReset.set(Value.kReverse);
				time = 150;
				if (time <= 50){
					robot.hal.latch.set(Value.kReverse);
				}
				else {
					time = 50;
					robot.hal.latch.set(Value.kReverse);
				}
			}
		}
			
		if (currentState == catapultState.START && nextState == catapultState.DUMP) {
			time = 100;
			robot.hal.latch.set(Value.kReverse);
			if (time <= 50){
			robot.hal.catapultReset.set(Value.kReverse);
			}
		}
		if ((currentState == catapultState.SHOOT || currentState == catapultState.DUMP) && nextState == catapultState.BRINGDOWN) {
			robot.hal.intake.set(Value.kForward);
			robot.hal.catapultReset.set(Value.kForward);
		}
		if (currentState == catapultState.BRINGDOWN && nextState == catapultState.LATCH) {
			time = 50;
			robot.hal.latch.set(Value.kForward);
		}
		if (currentState == catapultState.LATCH && nextState == catapultState.DONE) {
			robot.hal.intake.set(Value.kReverse);
		}
	}
	*/
	public void doNextState() {
		
	}
	
	public void reset() {
		currentState = catapultState.START;
	}
	
	
		
}
