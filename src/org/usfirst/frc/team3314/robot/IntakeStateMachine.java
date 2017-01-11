package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

enum intakeState {
	START,
	DROP,
	FORWARD,
	RETRACT,
	DONE
}



public class IntakeStateMachine {
	double TOLERANCE = .5;
	intakeState currentState;
	intakeState nextState;
	Robot robot;
	double time;
	boolean drop = false;
	boolean forward = false;
	
	public IntakeStateMachine(Robot myRobot, intakeState firstState) {
		robot = myRobot;
		currentState = firstState;
	}
	
	public void updateState() {
		calcNext();
		
		currentState = nextState;
	
	}
	
	public intakeState currentState() {
		
		return currentState;
		
	}
	
	public void calcNext() {
	 nextState = currentState;
	 if (robot.hi.getAutoIntake()){  
		 switch(currentState) {
		 case START:
			time = 50;
			if (robot.autoIntakeRequest) {
				drop = true;
				forward = true;
				if (robot.hal.intakeSensor.getVoltage() < 2){
					nextState = intakeState.RETRACT;
				}
			}
			break;
		 case RETRACT:
			time--;
			drop = false;
			if (time <= 0) {
				 nextState = intakeState.DONE;
			}
			break;
		 case DONE:
			 forward = false;
			 break; 
		 }
		 }
	 else {
		 if (!robot.hi.getAutoIntake()){
			 nextState = intakeState.START;
			 forward = false;
			 drop = false;
		 }
		
	 }
	}
	
	
	public void reset() {
		currentState = intakeState.START;
	}
	
	
}
