package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum autoNothingStates {
	START,
	DONE
}

public class AutoNothing implements Autonomous {
	
	autoNothingStates currentState;
	autoNothingStates nextState;
	Robot robot;
	

	public AutoNothing(Robot myRobot) {
		robot = myRobot;
		currentState = autoNothingStates.START;
		
	}
	
	public void reset() {
		currentState = autoNothingStates.START;
	}
	
	
	public void updateState() {
		
		calcNext();
		doTransition();
		
		currentState = nextState;
		
		SmartDashboard.putString("Auto State", currentState.toString());
	}
	
	public void calcNext() {
		nextState = currentState;
		
		switch (currentState) {
		case START:
			nextState = autoNothingStates.DONE;
			break;
		case DONE:
			break;
		}
	}
	
	public void doTransition() {
	
	}
}
