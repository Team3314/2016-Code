package org.usfirst.frc.team3314.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum auto30PtsStates {
	START,
	MOVEINTAKE,
	DRIVE,
	STOP,
	CALCANGLE1,
	GYROTURN1,
	DWELL1,
	ADJUSTDISTANCE,
	CALCANGLE2,
	GYROTURN2,
	DWELL2,
	CHECKANGLE,
	SHOOT,
	ANGLE,
	STOP2,
	REVERSE1,
	STOP3,
	ANGLE2,
	STOP4,
	REVERSE2,
	WAIT,
	INTAKEUP,
	STOPROLLER,
	DRIVE2,
	STOP5,
	CALCANGLE3,
	GYROTURN3,
	DWELL3,
	ADJUSTDISTANCE2,
	CALCANGLE4,
	GYROTURN4,
	DWELL4,
	CHECKANGLE2,
	SHOOT2,
	DONE
}

public class Auto30Pts implements Autonomous {
	
	auto30PtsStates currentState;
	auto30PtsStates nextState;
	Robot robot;
	double tolerance = 5;
	double distance;
	double desiredDistance;
	double time = 0;
	double firstAngle;
	double secondAngle;
	double distanceBack;
	double desiredAngle;
	double cameraError;
	double xAngleError;
	double angleAccum;
	double intAngle;

	public Auto30Pts(Robot myRobot) {
		
		robot = myRobot;
		currentState = auto30PtsStates.START;
		angleAccum = 0;
	}
	
	public void reset() {
		currentState = auto30PtsStates.START;
	}
	
	
	public void updateState() {
		distance = robot.tankDrive.getDistance();
		
		
		robot.camera.DisplayData();
		
		cameraError = robot.camera.centerX - 358;
		
		calcNext();
		doTransition();
		
		currentState = nextState;
		time --;
		
		SmartDashboard.putString("Auto State", currentState.toString());
		SmartDashboard.putNumber("Distance back", distanceBack);
		SmartDashboard.putNumber("AUTO DISTANCE", distance);
		
	}
	
	public void calcNext() {
		nextState = currentState;
		
		switch (currentState) {
		case START:
			angleAccum = 0;
			robot.hal.gyro.reset();
			intAngle = robot.hal.gyro.angle();
			nextState = auto30PtsStates.MOVEINTAKE;
			break;
		case MOVEINTAKE:
			if (time <= 0 && robot.hal.intake.get().toString() == "kForward") {
				nextState = auto30PtsStates.DRIVE;
			}
			break;
		/*case RESET:
			if (time <= 0 && robot.hal.catapultReset.get().toString() == "kForward" && !robot.hal.latchSensor.get()
			  && robot.hal.catapultAdjuster.get().toString() == "kReverse") {
				nextState = auto30PtsStates.DRIVE;
			}
			break;*/
		case DRIVE:
			if (distance >= desiredDistance ){//desiredDistance - distance <= tolerance) {
				nextState = auto30PtsStates.STOP;
			}
			break;
		case STOP:
			if (time <= 0 ){
				nextState = auto30PtsStates.CALCANGLE1;
			}
			break;
		case CALCANGLE1:
			// xAngleError = cameraError * (Width of view in Degrees / width of view in pixels)
			xAngleError = cameraError * .09375;
			firstAngle = xAngleError;
			robot.hal.gyro.reset();
			nextState = auto30PtsStates.GYROTURN1;
			break;
		case GYROTURN1:
			if ((Math.abs(xAngleError - robot.hal.gyro.angle())) < 2.75){
				nextState = auto30PtsStates.DWELL1;
			}
			break;
		case DWELL1:
			if (time <= 0){
				nextState = auto30PtsStates.ADJUSTDISTANCE;
			}
			break;
		case ADJUSTDISTANCE:
			if (robot.camera.centerY > 175 && robot.camera.centerY < 205){
				distanceBack = robot.tankDrive.getDistance();
				nextState = auto30PtsStates.CALCANGLE2;
			}
			break;
		case CALCANGLE2:
			xAngleError = cameraError * .09375;
			secondAngle = xAngleError;
			angleAccum += secondAngle;
			robot.hal.gyro.reset();
			nextState = auto30PtsStates.GYROTURN2;
			break;
		case GYROTURN2:
			if ((xAngleError - robot.hal.gyro.angle()) < Math.abs(2)){
				nextState = auto30PtsStates.DWELL2;
			}
			break;
		case DWELL2:
			if (time <= 0){	
				nextState = auto30PtsStates.CHECKANGLE;
			}
			break;
		case CHECKANGLE:
			xAngleError = cameraError * .09375;
			if (Math.abs(xAngleError) <= 2){
				nextState = auto30PtsStates.SHOOT;
			}
			else {
				nextState = auto30PtsStates.CALCANGLE2;
			}
			break;
		case SHOOT:
			if (robot.hal.latch.get().toString() == "kReverse"){
				nextState = auto30PtsStates.ANGLE;
			}
			break;
		case ANGLE:
			if (Math.abs(robot.tankDrive.desiredAngle - robot.hal.gyro.angle()) < 3){
				nextState = auto30PtsStates.STOP2;
			}
			break;
		case STOP2:
			if (time <= 0 ){
				nextState = auto30PtsStates.REVERSE1;
			}
			break;
		case REVERSE1:
			robot.hal.catapultReset.set(Value.kForward);
			if (robot.tankDrive.getDistance() < -distanceBack){
				nextState = auto30PtsStates.STOP3;
			}
			break;
		case STOP3:
			if (time <= 0 ){
				nextState = auto30PtsStates.ANGLE2;
			}
			break;
		case ANGLE2:
			if (Math.abs(robot.tankDrive.desiredAngle - robot.hal.gyro.angle()) < 3.25){
				nextState = auto30PtsStates.STOP4;
			}
			break;
		case STOP4:
			if (time <= 0 ){
				nextState = auto30PtsStates.REVERSE2;
			}
			break;
		case REVERSE2:
			if (distance <= desiredDistance){
				nextState = auto30PtsStates.WAIT;
			}
			break;
		case WAIT:
			if (time <= 0){
				nextState = auto30PtsStates.INTAKEUP;
			}
			break;
		case INTAKEUP:
			if(time <= 0) {
				nextState = auto30PtsStates.STOPROLLER;
			}
			break;
		case STOPROLLER:
			if (time <= 0) {
				nextState = auto30PtsStates.DRIVE2;
			}
			break;
		case DRIVE2:
			if (distance > 0.){
				robot.hal.intake.set(Value.kReverse);
				if (distance >= desiredDistance) {	
					nextState = auto30PtsStates.STOP5;
				}
			}
			break;
		case STOP5:
			if (time <= 0 ){
				nextState = auto30PtsStates.CALCANGLE3;
			}
			break;
		case CALCANGLE3:
			// xAngleError = cameraError * (Width of view in Degrees / width of view in pixels)
			xAngleError = cameraError * .09375;
			firstAngle = xAngleError;
			robot.hal.gyro.reset();
			nextState = auto30PtsStates.GYROTURN3;
			break;
		case GYROTURN3:
			if ((Math.abs(xAngleError - robot.hal.gyro.angle())) < 2.75){
				nextState = auto30PtsStates.DWELL3;
			}
			break;
		case DWELL3:
			if (time <= 0){
				nextState = auto30PtsStates.ADJUSTDISTANCE2;
			}
			break;
		case ADJUSTDISTANCE2:
			if (robot.camera.centerY > 175 && robot.camera.centerY < 205
					){
				distanceBack = robot.tankDrive.getDistance();
				nextState = auto30PtsStates.CALCANGLE4;
			}
			break;
		case CALCANGLE4:
			xAngleError = cameraError * .09375;
			secondAngle = xAngleError;
			angleAccum += secondAngle;
			robot.hal.gyro.reset();
			nextState = auto30PtsStates.GYROTURN4;
			break;
		case GYROTURN4:
			if ((xAngleError - robot.hal.gyro.angle()) < Math.abs(2.75)){
				nextState = auto30PtsStates.DWELL4;
			}
			break;
		case DWELL4:
			if (time <= 0){	
				nextState = auto30PtsStates.CHECKANGLE2;
			}
			break;
		case CHECKANGLE2:
			xAngleError = cameraError * .09375;
			if (Math.abs(xAngleError) <= 2.75){
				nextState = auto30PtsStates.SHOOT2;
			}
			else {
				nextState = auto30PtsStates.CALCANGLE4;
			}
			break;
		case SHOOT2:
			if (robot.hal.latch.get().toString() == "kReverse"){
				nextState = auto30PtsStates.DONE;
			}
			break;
		case DONE:
			
			break;
		}
	}
	
	public void doTransition() {
		if (currentState == auto30PtsStates.START && nextState == auto30PtsStates.MOVEINTAKE) {
		
			robot.hal.intake.set(Value.kForward);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 12;
		}
		
		/*if (currentState == auto30PtsStates.MOVEINTAKE && nextState == auto30PtsStates.RESET) {
			robot.hal.catapultReset.set(Value.kForward);
			robot.hal.catapultAdjuster.set(Value.kReverse);
			if (!robot.hal.latchSensor.get()){
				robot.hal.latch.set(Value.kForward);
			}
			time = 50;
		}*/
		
		if (currentState == auto30PtsStates.MOVEINTAKE && nextState == auto30PtsStates.DRIVE) {
			desiredDistance = 2.6;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(0);
			robot.tankDrive.setDriveTrainSpeed(-1);
		}
		
		if (currentState == auto30PtsStates.DRIVE && nextState == auto30PtsStates.STOP) {
			robot.cockRequest = true;
			robot.tankDrive.setDriveTrainSpeed(0);
			
			time = 12;
			robot.tankDrive.setDriveMode(driveMode.TANK);
		}
		
		if (currentState == auto30PtsStates.STOP && nextState == auto30PtsStates.CALCANGLE1) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		
		}
		
		if (currentState == auto30PtsStates.CALCANGLE1 && nextState == auto30PtsStates.GYROTURN1) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(xAngleError);
			
			
		}
		
		if (currentState == auto30PtsStates.GYROTURN1 && nextState == auto30PtsStates.DWELL1) {
			time = 25;
		}
		
		if (currentState == auto30PtsStates.DWELL1 && nextState == auto30PtsStates.ADJUSTDISTANCE) {
		//	time = 250;
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.camera.adjustDistance();
		}
		
		if (currentState == auto30PtsStates.ADJUSTDISTANCE && nextState == auto30PtsStates.CALCANGLE2) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		}
		
		if (currentState == auto30PtsStates.CALCANGLE2 && nextState == auto30PtsStates.GYROTURN2) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(xAngleError);
			
			
		}
		
		if (currentState == auto30PtsStates.GYROTURN2 && nextState == auto30PtsStates.DWELL2) {
			time = 10;
		}
		
		if (currentState == auto30PtsStates.DWELL2 && nextState == auto30PtsStates.CHECKANGLE) {
		}
		
		
		if (currentState == auto30PtsStates.CHECKANGLE && nextState == auto30PtsStates.SHOOT) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.hal.latch.set(Value.kReverse);
			time = 40;
		}
		
		if (currentState == auto30PtsStates.SHOOT && nextState == auto30PtsStates.ANGLE) {
			robot.shootRequest = false;
			robot.cockRequest = false;
			robot.catapultResetRequest = true;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(-angleAccum);
		}
		
		
		if (currentState == auto30PtsStates.ANGLE && nextState == auto30PtsStates.STOP2) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			time = 12;
			
		}
		
		if (currentState == auto30PtsStates.STOP2 && nextState == auto30PtsStates.REVERSE1) {
			
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(.75);
			robot.tankDrive.setDriveAngle(robot.hal.gyro.angle());
		}
		
		if (currentState == auto30PtsStates.REVERSE1 && nextState == auto30PtsStates.STOP3) {
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setStickInputs(0, 0);
			time = 2;
		}
		
		if (currentState == auto30PtsStates.STOP3 && nextState == auto30PtsStates.ANGLE2) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(0);
		}
		
		if (currentState == auto30PtsStates.ANGLE2 && nextState == auto30PtsStates.STOP4) {
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		}
		
		if (currentState == auto30PtsStates.STOP4 && nextState == auto30PtsStates.REVERSE2) {
			desiredDistance = -2.1 + distanceBack;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(.7);
			robot.tankDrive.setDriveAngle(intAngle+6);//(-4);
			robot.hal.intake.set(Value.kForward);
			robot.hal.intakeTalon.set(-1);
			time = 7;
			
		}
		if (currentState == auto30PtsStates.REVERSE2 && nextState == auto30PtsStates.WAIT) {
			time = 50;
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			
		}
		if (currentState == auto30PtsStates.WAIT && nextState == auto30PtsStates.INTAKEUP) {
		
			time = 12;
		}
		if (currentState == auto30PtsStates.INTAKEUP && nextState == auto30PtsStates.STOPROLLER){
			
			
		}
		if (currentState == auto30PtsStates.STOPROLLER && nextState == auto30PtsStates.DRIVE2) {
			robot.hal.latch.set(Value.kForward);
			robot.catapultController.reset();
			desiredDistance = 2.2;
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveAngle(robot.hal.gyro.angle());
			robot.tankDrive.setDriveTrainSpeed(-1);
			
		}
		
		if (currentState == auto30PtsStates.DRIVE2 && nextState == auto30PtsStates.STOP5) {
			robot.hal.intakeTalon.set(0);
			robot.cockRequest = true;
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.hal.intake.set(Value.kReverse);
			time = 25;
			robot.tankDrive.setDriveMode(driveMode.TANK);
			
		}
		
		if (currentState == auto30PtsStates.STOP5 && nextState == auto30PtsStates.CALCANGLE3) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		
		}
		
		if (currentState == auto30PtsStates.CALCANGLE3 && nextState == auto30PtsStates.GYROTURN3) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(xAngleError);
			
			
		}
		
		if (currentState == auto30PtsStates.GYROTURN3 && nextState == auto30PtsStates.DWELL3) {
			time = 12;
		}
		
		if (currentState == auto30PtsStates.DWELL3 && nextState == auto30PtsStates.ADJUSTDISTANCE2) {
			//time = 250;
			robot.tankDrive.leftDriveTalon1.setPosition(0);
			robot.tankDrive.rightDriveTalon1.setPosition(0);
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.camera.adjustDistance();
		}
		
		if (currentState == auto30PtsStates.ADJUSTDISTANCE2 && nextState == auto30PtsStates.CALCANGLE4) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		}
		
		if (currentState == auto30PtsStates.CALCANGLE4 && nextState == auto30PtsStates.GYROTURN4) {
			robot.tankDrive.setDriveMode(driveMode.GYROLOCK);
			robot.tankDrive.setDriveTrainSpeed(0);
			robot.tankDrive.setDriveAngle(xAngleError);
			
			
		}
		
		if (currentState == auto30PtsStates.GYROTURN4 && nextState == auto30PtsStates.DWELL4) {
			time = 12;
		}
		
		if (currentState == auto30PtsStates.DWELL4 && nextState == auto30PtsStates.CHECKANGLE2) {
		}
		
		
		if (currentState == auto30PtsStates.CHECKANGLE2 && nextState == auto30PtsStates.SHOOT2) {
			robot.hal.gyro.reset();
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
			robot.hal.latch.set(Value.kReverse);
			time = 50;
		}
		
		
		if (currentState == auto30PtsStates.SHOOT2 && nextState == auto30PtsStates.DONE) {
			robot.catapultResetRequest = false;
			robot.tankDrive.setDriveMode(driveMode.TANK);
			robot.tankDrive.setStickInputs(0, 0);
		}
	}
}
