package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.Joystick;

/*
 * DRIVER CONTROLS: 2 JOYSTICKS
 * RIGHT STICK - RIGHT TRACK
 * LEFT STICK - LEFT TRACK
 * AUTO INTAKE - LEFT TRACK BUTTON 5 
 * HIGH GEAR - LEFT TRACK BUTTON 3
 * LOW GEAR - LEFT TRACK BUTTON 2
 * COMPRESSOR OFF - RIGHT TRACK BUTTON 4
 * GYRO RESET - RIGHT TRACK TRIGGER
 * COAST MODE - RIGHT TRACK BUTTON 5
 * 
 * OPERATOR CONTROLS: XBOX CONTROLLER
 * SHOOT - RIGHT BUMPER
 * DUMP - LEFT BUMPER
 * INTAKE IN - RIGHT TRIGGER
 * INTAKE OUT - LEFT TRIGGER
 * MANUAL INTAKE DROP - A
 * AUTO INTAKE - B
 * PTO - START + SELECT
 * CATAPULT ADJUST - X
 * CATAPULT RESET - Y
 * COCK - LEFT STICK IN
 * UNLATCH - RIGHT STICK IN
 * EXTEND CLIMBER PNEUMATIC - START + RIGHT TRIGGER
 * RUN CLIMBER TALON FORWARD - DPAD UP
 * RUN CLIMBER TALON REVERSE & RETRACT CLIMBER PNEUMATIC - DPAD DOWN

*/
public class HumanInput {
	Joystick leftTrackStick;
	Joystick rightTrackStick;
	Joystick operator;
	
	boolean resultPTO = false;
	
	boolean previousPTO = false;
	boolean currentPTO = false;
	
	
	public HumanInput(){
		operator = new Joystick(3);
		rightTrackStick = new Joystick(0);
		leftTrackStick = new Joystick(1);
	}
	
	public boolean getBatterReverse() {
		boolean result = false;
		result = leftTrackStick.getRawButton(1);
		return result;
	}
	
	public boolean getGyroReset() {
		boolean result = false;
		result = leftTrackStick.getRawButton(7);
		return result;
	}
	
	public boolean getCoast() {
		boolean result = false;
		result = rightTrackStick.getRawButton(5);
		return result;
	}
	
	public boolean getShoot() {
		boolean result = false;
		result = operator.getRawButton(6);
		return result;
	}
	
	public boolean getPowerControl() {
		boolean triggered = false;
		boolean result = false;
		if (rightTrackStick.getRawAxis(2) <= 0.60){
			triggered = true;
		}
		result = triggered;
		return result;
	}
	
	public boolean getIntakeForward() {
		boolean triggered = false;
		boolean result = false;
		if (operator.getRawAxis(3) >= 0.75){
			triggered = true;
		}
		result = triggered;
		return result;
	}
	
	public boolean getIntakeReverse() {
		boolean triggered = false;
		boolean result = false;
		if (operator.getRawAxis(2) >= 0.75){
			triggered = true;
		}
		result = triggered;
		return result;
	}
	
	public boolean getAutoIntake() {
		boolean result = false;
		result = (leftTrackStick.getRawButton(5) || operator.getRawButton(2));
		return result;
	}  
	
	public boolean getHighGear() {
		boolean result;
		result = leftTrackStick.getRawButton(3);
		return result;
	}
	public boolean getLowGear() {
		boolean result;
		result = leftTrackStick.getRawButton(2);
		return result;
	}
	
	public boolean getPTO() {
		previousPTO = currentPTO;
		if (operator.getRawButton(7) && operator.getRawButton(8)) {
			currentPTO = true;
		}
		else {
			currentPTO = false;
		}
		if (currentPTO && !previousPTO) {
			resultPTO = !resultPTO;
		}
		return resultPTO;
	}
	
	public boolean getCatapultAdjuster() {
		boolean result;
		result = operator.getRawButton(3);
		return result;
	}
	
	public boolean getCatapultReset() {
		boolean result = false;
		result = operator.getRawButton(4);
		return result;
	}   
	
	
	
	public boolean getDump() {
		boolean result = false;
		result = operator.getRawButton(5);
		return result;
	}
	
	public boolean getCenterCamera() {
		boolean result = false;
		result = leftTrackStick.getRawButton(11);
		return result;
	}
	
	/*public boolean getAdjustDistance() {
		boolean result = false;
		result = rightTrackStick.getRawButton(1);
		return result;
	}*/
	
	public boolean getEncoderOverride() {
		boolean result = false;
		result = rightTrackStick.getRawButton(3);
		return result;
	}
	
	public boolean getIntakeOverride() {
		boolean result = false;
		result = operator.getRawButton(1);
		return result;
	}
	
	public boolean getCock() {
		boolean result = false;
		result = operator.getRawButton(9);
		return result;
	}
	public boolean getUnlatch() {
		boolean result = false;
		result = operator.getRawButton(10);
		return result;
	}
	
	public boolean getCompressor() {	
			boolean result = false;
			result = rightTrackStick.getRawButton(4);
			return result;
	}

	public boolean getSYMMETRIC() {
		boolean result = false;
		result = leftTrackStick.getRawButton(3);
		return result;
	}
	
	public boolean getGYROLOCK() {
		boolean result = false;
		result = rightTrackStick.getRawButton(1);
		return result;
	}
	
	public boolean getENCODERLOCK() {
		boolean result = false;
		result = leftTrackStick.getRawButton(7);
		return result;
	}
	
	public boolean getExtendClimberPneumatic() {
		boolean result = false;
		if (operator.getRawButton(8) && operator.getRawAxis(3) >= .75) {
			result = true;
		}
		return result;
	}
	
	public boolean getRetractClimberPneumatic() {
		boolean result = false;
		if (operator.getRawButton(7) && operator.getRawAxis(2) >= .75) {
			result = true;
		}
		return result;
	}
	
	public boolean getClimberTalon() {
		boolean result = false;
		if (operator.getPOV() == 0) {
			result = true;
		}
		return result;
	}
	
	public boolean getReverseClimberTalon() {
		boolean result = false;
		if (operator.getPOV() == 180) {
			result = true;
		}
		return result;
	}
	
	public boolean getRelayOn() {
		boolean result = false;
		result = rightTrackStick.getRawButton(3);
		return result;
	}

}
