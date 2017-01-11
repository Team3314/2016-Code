
package org.usfirst.frc.team3314.robot;


import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    
    CatapultStateMachine catapultController;
    IntakeStateMachine intakeController;
    BatterReverseStateMachine batterController;
    
   // Autonomous auto;
    
   // AutoCrossDefense autoOp1;
  //  AutoReverseLowGoal autoOp2;
    
    AutoNothing auto0;
    AutoCrossDefense auto1;
    AutoSpyBoxShootHigh auto2;
    AutoCrossCenterShootHigh auto3;
    AutoDefense5HighGoal auto4;
    AutoLowBarHighGoal auto5;
    Auto30Pts auto6;
    AutoShootHighCrossBackReset auto7;
    
    HardwareAbstractionLayer hal;
    HumanInput hi;
    CustomCamera camera;
    TankDriveTrain tankDrive;
//	CameraServer2 server;
    
    SendableChooser autoSelect;
    
    
    boolean shootRequest;
    boolean intakeForwardRequest;
    boolean intakeReverseRequest;
    boolean highGearRequest;
    boolean lowGearRequest;
    boolean ptoRequest;
	boolean catapultAdjustRequest;
	boolean dumpRequest;
	boolean centerCameraRequest;
	boolean adjustDistanceRequest;
	boolean autoIntakeRequest;
	boolean intakeOverrideRequest;
	boolean catapultResetRequest;
    boolean cockRequest;
    boolean encoderOverrideRequest;
    boolean intakeOverride = false;
    boolean intake = false;
    boolean previousIntake;
    boolean unlatchRequest;
    boolean SYMMETRICRequest;
    boolean GYROLOCKRequest;
    boolean ENCODERRequest;
    boolean	gyroResetRequest;
    boolean compressorOffRequest;
    boolean powerControlRequest = false;
    boolean coastRequest;
    boolean batterReverseRequest;
    boolean climberTalonForwardRequest;
    boolean climberTalonReverseRequest;
    boolean extendClimberPneumaticRequest;
    boolean retractClimberPneumaticRequest;
    boolean relayOnRequest;
    
    boolean lastENCODERLOCK = false;
    boolean lastGYROLOCK = false;
    boolean lastCatapultAdjust = false;
    boolean doUpdate = true;
    boolean doIntake = true;
    
    
    boolean latched = false;
    
      
    double leftSpeed = 0;
  	double rightSpeed = 0;
  	double driveEncoderScale;
	
    public void robotInit() {
    	
        // autoSelect = new SendableChooser();
        // autoSelect.addObject("CROSS DEFENSE", new AutoCrossDefense(this));
        // autoSelect.addObject("SPY BOX HIGH GOAL CROSS LOWBAR", new AutoSpyBotHighGoalReverseCrossLowBar(this));
    	// autoOp1 = new AutoCrossDefense(this);
        // autoOp2 = new AutoReverseLowGoal(this); 
        // SmartDashboard.putData("Auto Choices", autoSelect);
        
    	auto0 = new AutoNothing(this);
    	auto1 = new AutoCrossDefense(this);
        auto2 = new AutoSpyBoxShootHigh(this);
        auto3 = new AutoCrossCenterShootHigh(this);
        auto4 = new AutoDefense5HighGoal(this);
        auto5 = new AutoLowBarHighGoal(this);
        auto6 = new Auto30Pts(this);
        auto7 = new AutoShootHighCrossBackReset(this);
        
        catapultController = new CatapultStateMachine(this, catapultState.START);
        intakeController = new IntakeStateMachine(this, intakeState.START);
        batterController = new BatterReverseStateMachine(this, batterReverseStates.START);
        
      
        
        
        hal = new HardwareAbstractionLayer(this);
        hi = new HumanInput();
        camera = new CustomCamera(this);
        tankDrive = new TankDriveTrain(this);
           
        camera.CreateImage();
        camera.CreateServer();
        
     /*   
        server = CameraServer2.getInstance();
        server.setQuality(50);
		server.startAutomaticCapture("cam1");*/
		
		hal.pto.set(Value.kForward);
		hal.climber.set(Value.kForward);
        
    }
    
	
    public void autonomousInit() {
    	hal.flashlight.set(Relay.Value.kReverse);
		hal.pto.set(Value.kForward);
		hal.climber.set(Value.kForward);
    	hal.driveShifter.set(Value.kForward);
    	//autoOp1.reset();
    	//autoOp2.reset();
    	powerControlRequest = true;
    	
    	//if (powerControlRequest){
    		tankDrive.leftDriveTalon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    		tankDrive.rightDriveTalon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	//}
    	
    	//auto = (Autonomous) autoSelect.getSelected();
    	if (hal.autoSelect1.get() && hal.autoSelect2.get() && hal.autoSelect4.get()){
    		auto0.reset();
    	}
    	if (!hal.autoSelect1.get() && hal.autoSelect2.get() && hal.autoSelect4.get()){
    		auto1.reset();
    	}
    	else if (hal.autoSelect1.get() && !hal.autoSelect2.get() && hal.autoSelect4.get()){
    		auto2.reset();
    	}
    	else if (!hal.autoSelect1.get() && !hal.autoSelect2.get() && hal.autoSelect4.get()){
    		auto3.reset();
    	}
    	else if (hal.autoSelect1.get() && hal.autoSelect2.get() && !hal.autoSelect4.get()){
    		auto4.reset();
    	}
    	else if (!hal.autoSelect1.get() && hal.autoSelect2.get() && !hal.autoSelect4.get()){
    		auto5.reset();
    	}
    	else if (hal.autoSelect1.get() && !hal.autoSelect2.get() && !hal.autoSelect4.get()){
    		auto6.reset();
    	}
    	else if (!hal.autoSelect1.get() && !hal.autoSelect2.get() && !hal.autoSelect4.get()){
    		auto7.reset();
    	}
   
    }

   
    public void autonomousPeriodic() {
    	camera.DisplayData();
    	hal.gyro.update();
    	//updateButtonStatus();
    	//autoOp1.updateState();
    	//autoOp2.updateState();
    	//auto1.updateState();
    	tankDrive.update();
    	catapultController.updateState();
    	if (hal.autoSelect1.get() && hal.autoSelect2.get() && hal.autoSelect4.get()){
    		auto0.updateState();
    	}
    	else if (!hal.autoSelect1.get() && hal.autoSelect2.get() && hal.autoSelect4.get()){
    		auto1.updateState();
    	}
    	else if (hal.autoSelect1.get() && !hal.autoSelect2.get() && hal.autoSelect4.get()){
    		auto2.updateState();
    	}
    	else if (!hal.autoSelect1.get() && !hal.autoSelect2.get() && hal.autoSelect4.get()){
    		auto3.updateState();
    	}
    	else if (hal.autoSelect1.get() && hal.autoSelect2.get() && !hal.autoSelect4.get()){
    		auto4.updateState();
    	}
    	else if (!hal.autoSelect1.get() && hal.autoSelect2.get() && !hal.autoSelect4.get()){
    		auto5.updateState();
    	}
    	else if (hal.autoSelect1.get() && !hal.autoSelect2.get() && !hal.autoSelect4.get()){
    		auto6.updateState();
    	}
    	else if (!hal.autoSelect1.get() && !hal.autoSelect2.get() && !hal.autoSelect4.get()){
    		auto7.updateState();
    	}
   
    	SmartDashboard.putNumber("centerx", camera.centerX);
    	SmartDashboard.putBoolean("DIO 7", hal.autoSelect1.get());
    	SmartDashboard.putBoolean("DIO 8", hal.autoSelect2.get());
    	SmartDashboard.putBoolean("DIO 9", hal.autoSelect4.get());
    	SmartDashboard.putNumber("Desired Distance", batterController.desiredDistance);
		SmartDashboard.putNumber("distance", tankDrive.getDistance());
    }
    
    public void teleopInit(){
    	hal.gyro.calibrate();
		hal.pto.set(Value.kForward);
    	hal.driveShifter.set(Value.kReverse);
    	hal.climber.set(Value.kForward);
    	tankDrive.leftDriveTalon1.setPosition(0);
    	tankDrive.rightDriveTalon1.setPosition(0);
    }

    
    public void teleopPeriodic() {
    	hal.gyro.update();
    	camera.DisplayData();
    	
    	updateButtonStatus();
    	catapultController.updateState();
    	intakeController.updateState();
    	
    	if (gyroResetRequest){
    		hal.gyro.reset();
    	}
    	
    	if (coastRequest){
    		
    		tankDrive.rightDriveTalon1.enableBrakeMode(false);
    		tankDrive.rightDriveTalon2.enableBrakeMode(false);
    		tankDrive.rightDriveTalon3.enableBrakeMode(false);
    		
    		tankDrive.leftDriveTalon1.enableBrakeMode(false);
    		tankDrive.leftDriveTalon2.enableBrakeMode(false);
    		tankDrive.leftDriveTalon3.enableBrakeMode(false);
    		
    	} else {
    		
    		tankDrive.rightDriveTalon1.enableBrakeMode(true);
    		tankDrive.rightDriveTalon2.enableBrakeMode(true);
    		tankDrive.rightDriveTalon3.enableBrakeMode(true);
    		
    		tankDrive.leftDriveTalon1.enableBrakeMode(true);
    		tankDrive.leftDriveTalon2.enableBrakeMode(true);
    		tankDrive.leftDriveTalon3.enableBrakeMode(true);
    		
    	}
    	
    	if (compressorOffRequest){
    		hal.pcm1.stop();
    	}	
    	else {
    		hal.pcm1.start();
    	}
 
    	if (unlatchRequest) {
    		hal.latch.set(Value.kReverse);
    	}
    	if (highGearRequest){
    		hal.driveShifter.set(Value.kReverse);
    	}
    	if (lowGearRequest) {
    		hal.driveShifter.set(Value.kForward);
    	}
    	
    	if (ptoRequest){
    		hal.pto.set(Value.kReverse);
    	}
    	else {
    		hal.pto.set(Value.kForward);
    	}
    	
    	if (catapultAdjustRequest && !lastCatapultAdjust){
    		if (hal.catapultAdjuster.get().toString() == "kForward") {
    			hal.catapultAdjuster.set(Value.kReverse);
    		}
    		if (hal.catapultAdjuster.get().toString() == "kReverse" || hal.catapultAdjuster.get().toString() == "kOff") {
    			hal.catapultAdjuster.set(Value.kForward);
    		}
    	}
    	
    	
    	if (catapultResetRequest){
    		catapultController.currentState = catapultState.BRINGDOWN;
    	}
    	
    	if (catapultController.currentState != catapultState.COCK && catapultController.currentState != catapultState.CHECKCOCK
    	   && catapultController.currentState != catapultState.SHOOT && catapultController.currentState != catapultState.COCKED){
	    	if (hi.getIntakeOverride() || intakeController.drop){
	    		hal.intake.set(Value.kForward);
	    	}
	    	else {
	    		hal.intake.set(Value.kReverse);
	    	}
    	}
    	
    	if ((hi.getIntakeForward() || intakeController.forward) && doIntake){
    		hal.intakeTalon.set(-1);
    	}
    	else if (hi.getIntakeReverse() && doIntake){
			 hal.intakeTalon.set(1);
		 }
    	else {
    		hal.intakeTalon.set(0);
    	}
    	
    	
    	if (!relayOnRequest || !batterReverseRequest || !hi.operator.getRawButton(8) || !hi.leftTrackStick.getRawButton(1)){
    		hal.flashlight.set(Relay.Value.kOff);
    	}
    	
    	 
    	if (hi.operator.getRawButton(8) || hi.leftTrackStick.getRawButton(1) || relayOnRequest) {
    		hal.flashlight.set(Relay.Value.kForward);
    		//hal.flashlight.set(Relay.Value.kOff);
    		//hal.pcm2.stop();
    	}
    	
    	if (hi.operator.getRawButton(7)){
    		hal.flashlight.set(Relay.Value.kReverse);
    	}
    	
    	/*
    	if (extendClimberPneumaticRequest) {
    		hal.climber.set(Value.kForward);
    	}
    	
    	
    	if (climberTalonForwardRequest && !ptoRequest) {
    		hal.climberTalon.set(.5);
    	}
    	else if (climberTalonReverseRequest) {
    		hal.climberTalon.set(-.5);
    		hal.climber.set(Value.kReverse);
    	}
    	else if (ptoRequest && climberTalonForwardRequest) {
    		doUpdate = false;
    		hal.climberTalon.set(.2);
    		tankDrive.setDriveTrainSpeed(-.2);
    	}
    	else {
    		hal.climberTalon.set(0);
    		doUpdate = true;
    	}
    	*/
    	
    	if (extendClimberPneumaticRequest) {
    		doIntake = false;
    		hal.climber.set(Value.kForward);
    	}
    	else {
    		doIntake = true;
    	}
    	if (retractClimberPneumaticRequest) {
    		doIntake = false;
    		hal.climber.set(Value.kReverse);
    	}
    	else {
    		doIntake = true;
    	}
    	
    	
    	tankDrive.setStickInputs(hi.leftTrackStick.getY(), hi.rightTrackStick.getY()); 
    	
    	//tankDrive.setDriveMode(hi.rightTrackStick.getRawButton(3) ? driveMode.SYMMETRIC: driveMode.TANK);
    	
    	if (SYMMETRICRequest || ptoRequest){
    		tankDrive.setDriveMode(driveMode.SYMMETRIC);
    		hal.climber.set(Value.kForward);
    	}
    	else if (GYROLOCKRequest){
    		if (!lastGYROLOCK){
    			tankDrive.setDriveAngle(hal.gyro.angle());
    			tankDrive.setDriveMode(driveMode.GYROLOCK);
    		}
    		tankDrive.setDriveTrainSpeed(hi.leftTrackStick.getY());
    	}
    	else if (ENCODERRequest){
    		if (!lastENCODERLOCK){
    			tankDrive.setDriveMode(driveMode.ENCODERLOCK);
    		}
    		tankDrive.setDriveTrainSpeed(hi.leftTrackStick.getY());
    	} else if (doUpdate) {
    		tankDrive.setDriveMode(driveMode.TANK);
    	}
    	
    	if (powerControlRequest){
    		tankDrive.leftDriveTalon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    		tankDrive.rightDriveTalon1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    	}
    	
    	if (centerCameraRequest){
    		//doUpdate = false;
    		camera.centerCamera();
    	}
    	
    	if (hi.leftTrackStick.getRawButton(10)){
    		camera.adjustDistance();
    	}
    	
    	/*if (batterReverseRequest){
    		doUpdate = false;
    		batterController.updateState();
    	}
    	else {
    		doUpdate = true;
    		batterController.reset();
    	}*/
    	
    	if (doUpdate){
    		tankDrive.update();
    	}
    	
    	lastGYROLOCK = GYROLOCKRequest;
    	lastENCODERLOCK = ENCODERRequest;
    	lastCatapultAdjust = catapultAdjustRequest;
    	
    	//
    	SmartDashboard.putNumber("stick direction", tankDrive.leftStickInput);
    	
    	SmartDashboard.putNumber("Left Speed", tankDrive.rawLeftSpeed);
    	SmartDashboard.putNumber("Right Speed", tankDrive.rawRightSpeed);
    	
    	SmartDashboard.putNumber("centerx", camera.centerX);
    	SmartDashboard.putBoolean("Do Intake", doIntake);
    	SmartDashboard.putBoolean("DIO 7", hal.autoSelect1.get());
    	SmartDashboard.putBoolean("DIO 8", hal.autoSelect2.get());
    	SmartDashboard.putBoolean("DIO 9", hal.autoSelect4.get());
    	
    	SmartDashboard.putNumber("intakeSensor volts", hal.intakeSensor.getVoltage());
    	
    	SmartDashboard.putNumber("right Stick magnitude", hi.rightTrackStick.getMagnitude());
    	SmartDashboard.putNumber("left Stick magnitude", hi.leftTrackStick.getMagnitude());
    	SmartDashboard.putNumber("Left speed", tankDrive.leftDriveTalon1.getSpeed());
    	SmartDashboard.putNumber("Right speed", tankDrive.rightDriveTalon1.getSpeed());
    	SmartDashboard.putNumber("counter", tankDrive.caseCounter);
    	SmartDashboard.putNumber("Right encoder error", tankDrive.rightDriveTalon1.getClosedLoopError());
    	SmartDashboard.putNumber("Left encoder error", tankDrive.leftDriveTalon1.getClosedLoopError());
    	SmartDashboard.putNumber("R AW RIGHT SPEED", tankDrive.rawRightSpeed);
    	SmartDashboard.putNumber("RAW LEFT SPEED", tankDrive.rawLeftSpeed);
    	SmartDashboard.putNumber("POV", hi.operator.getPOV());
    	SmartDashboard.putBoolean("CATAPULT RESET SWITCH", hal.catapultResetSensor.get());
    	SmartDashboard.putBoolean("Do Update?", doUpdate);
    	SmartDashboard.putBoolean("PTOREQUEST", ptoRequest);
    	SmartDashboard.putString("PTO", hal.pto.get().toString());
    	
    	SmartDashboard.putNumber("Left 1 current", tankDrive.leftDriveTalon1.getOutputCurrent());
    	SmartDashboard.putNumber("Left 2 current", tankDrive.leftDriveTalon2.getOutputCurrent());
    	SmartDashboard.putNumber("Left 3 current", tankDrive.leftDriveTalon3.getOutputCurrent());
    	
    	SmartDashboard.putNumber("Right 1 current", tankDrive.rightDriveTalon1.getOutputCurrent());
    	SmartDashboard.putNumber("Right 2 current", tankDrive.rightDriveTalon2.getOutputCurrent());
    	SmartDashboard.putNumber("Right 3 current", tankDrive.rightDriveTalon3.getOutputCurrent());
    	
    	SmartDashboard.putNumber("gyro", hal.gyro.angle());
    	
    	SmartDashboard.putNumber("Desired Distance", batterController.desiredDistance);
		SmartDashboard.putNumber("distance", tankDrive.getDistance());
		
		SmartDashboard.putNumber("Left Encoder Position", tankDrive.leftDriveTalon1.getPosition());
    	SmartDashboard.putNumber("Right Encoder Position", tankDrive.rightDriveTalon1.getPosition());
    	
    	SmartDashboard.putString("Drive State", tankDrive.currentMode.toString());
    	SmartDashboard.putString("catapult state", catapultController.getCurrentState().toString());	
    
    	SmartDashboard.putBoolean("DIO 7", hal.autoSelect1.get());
    	SmartDashboard.putBoolean("DIO 8", hal.autoSelect2.get());
    	SmartDashboard.putBoolean("DIO 9", hal.autoSelect4.get());
    	SmartDashboard.putNumber("First Angle", auto7.firstAngle);
    	SmartDashboard.putNumber("Gyro temp", hal.gyro.temp());
    }
    	
    	
 /*   SmartDashboard.putString("catapult state", catapultController.currentState.toString());	
	SmartDashboard.putString("Cocked state", hal.catapultReset.get().toString());		
     SmartDashboard.putBoolean("compressor enabled", hal.pcm1.enabled());
     SmartDashboard.putBoolean("pressure switch", hal.pcm1.getPressureSwitchValue());
     SmartDashboard.putNumber("current", hal.pcm1.getCompressorCurrent());
     SmartDashboard.putNumber("gyro", hal.gyro.angle());
     SmartDashboard.putBoolean("forwarhtd", intakeController.forward);
     SmartDashboard.putBoolean("drop", intakeController.drop);
     
 
   //  SmartDashboard.putBoolean("micro", hal.latchSensor.get());
     SmartDashboard.putNumber("Intake Sensor", hal.intakeSensor.getVoltage());
     
     SmartDashboard.putBoolean("Intake override", hi.getIntakeOverride());
     SmartDashboard.putBoolean("Dropped", intakeController.drop);
     SmartDashboard.putNumber("time", intakeController.time);
     
     SmartDashboard.putString("Intake State", intakeController.currentState.toString());
     SmartDashboard.putString("Intake", hal.intake.get().toString());
     
     SmartDashboard.putNumber("Distance in inches", camera.getDistance());
    	SmartDashboard.putNumber("FOV", camera.getFOV());
    	SmartDashboard.putNumber("Horizontal Angle", camera.getHorizontalAngle());
    	SmartDashboard.putNumber("Veritcal Angle", camera.getVerticalAngle());

    	
     
    	}
    	
     
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        SmartDashboard.putNumber("FOV", camera.getFOV());
    }
    
    public void updateButtonStatus() {
    	shootRequest = hi.getShoot();
    	intakeForwardRequest = hi.getIntakeForward();
    	intakeReverseRequest = hi.getIntakeReverse();
    	highGearRequest = hi.getHighGear();
    	lowGearRequest = hi.getLowGear();  
    	ptoRequest = hi.getPTO();
    	catapultAdjustRequest = hi.getCatapultAdjuster();
    	dumpRequest = hi.getDump();
    	centerCameraRequest = hi.getCenterCamera();
    	//adjustDistanceRequest = hi.getAdjustDistance();
    	autoIntakeRequest = hi.getAutoIntake();
    	intakeOverrideRequest = hi.getIntakeOverride();
    	catapultResetRequest = hi.getCatapultReset();
    	encoderOverrideRequest = hi.getEncoderOverride();
    	cockRequest = hi.getCock();
    	unlatchRequest = hi.getUnlatch();
    	SYMMETRICRequest = hi.getSYMMETRIC();
    	GYROLOCKRequest = hi.getGYROLOCK();
    	ENCODERRequest = hi.getENCODERLOCK();
    	gyroResetRequest = hi.getGyroReset();
    	compressorOffRequest = hi.getCompressor();
    	powerControlRequest = hi.getPowerControl();
    	coastRequest = hi.getCoast();
    	batterReverseRequest = hi.getBatterReverse();
    	climberTalonForwardRequest = hi.getClimberTalon();
    	climberTalonReverseRequest = hi.getReverseClimberTalon();
    	extendClimberPneumaticRequest = hi.getExtendClimberPneumatic();
    	retractClimberPneumaticRequest = hi.getRetractClimberPneumatic();
    	relayOnRequest = hi.getRelayOn();
    }
    
}
