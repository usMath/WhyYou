package robot;

import ccre.channel.BooleanCell;
import ccre.channel.BooleanInput;
import ccre.channel.BooleanOutput;
import ccre.channel.EventOutput;
import ccre.channel.FloatInput;
import ccre.channel.FloatOutput;
import ccre.ctrl.ExtendedMotor.OutputControlMode;
import ccre.ctrl.ExtendedMotorFailureException;
import ccre.ctrl.binding.ControlBindingCreator;
import ccre.drivers.ctre.talon.TalonExtendedMotor;
import ccre.frc.FRC;
import ccre.frc.FRCApplication;
import ccre.instinct.InstinctModule;
import ccre.log.Logger;

/**
 * This is the core class of a CCRE project. The CCRE launching system will make
 * sure that this class is loaded, and will have set up everything else before
 * loading it. If you change the name, use Eclipse's rename functionality. If
 * you don't, you will have to change the name in Deployment.java.
 *
 * Make sure to set {@link #TEAM_NUMBER} to your team number.
 */
public class RobotTemplate implements FRCApplication {

	/**
	 * This is where you specify your team number. It is used to find your
	 * roboRIO when you download code.
	 */
	public static final int TEAM_NUMBER = 1540;

	@Override
	public void setupRobot() throws ExtendedMotorFailureException {

		Logger.info("You v0.71 2016-12-29");
		
		//Control Binding
		final ControlBindingCreator controlBinding = FRC.controlBinding();
		
		// Right drive train (4, 5, 6)
		FloatOutput right1 = FRC.talonCAN(4).simpleControl();
		FloatOutput right2 = FRC.talonCAN(5).simpleControl();
		FloatOutput right3 = FRC.talonCAN(6).simpleControl();
		
		FloatOutput right = right1.combine(right2).combine(right3).addRamping(0.02f, FRC.constantPeriodic);
		
		// Left drive train (1, 2, 3)
		FloatOutput left1 = FRC.talonCAN(1).simpleControl();
		FloatOutput left2 = FRC.talonCAN(2).simpleControl();
		FloatOutput left3 = FRC.talonCAN(3).simpleControl();
		
		FloatOutput left = left1.combine(left2).combine(left3).addRamping(0.02f, FRC.constantPeriodic).negate();
		
		// Shooter
		FloatOutput flywheel = FRC.talon(0);
		FloatOutput actuator = FRC.talon(1);
		
		// Combined drive train
		FloatOutput drive = left.combine(right);
    	FloatOutput turnLeft = right.combine(left.negate());
    	FloatOutput turnRight = left.combine(right.negate());
		
		// Arm has lower joint, upper joint, and claw. Similar to human arm (hand = claw).
		FloatOutput armJoint = FRC.talon(2);
		BooleanOutput clawA = FRC.solenoid(4);
		BooleanOutput clawB = FRC.solenoid(5);
		
		//Controllers
		
		//Driver - Movement
		
		//Left side
    	FloatInput leftTrain = controlBinding.addFloat("Left drive train").deadzone(0.2f);
    	//Right side
    	FloatInput rightTrain = controlBinding.addFloat("Right drive train").deadzone(0.2f);
    	// Move backwards
    	FloatInput backwards = controlBinding.addFloat("Backwards").deadzone(0.2f);
    	// Move forwards
    	FloatInput forwards = controlBinding.addFloat("Forwards").deadzone(0.2f);
    	
    	//Copilot - Arm and Shooting
    	
    	//Arm
    	FloatInput arm = controlBinding.addFloat("Arm").deadzone(0.1f);
    	BooleanInput armButtonOn = controlBinding.addBoolean("Arm activation");
    	BooleanInput armButtonHigh = controlBinding.addBoolean("Raise Arm");
    	//Claw
    	BooleanInput buttonO = controlBinding.addBoolean("Claw activation");
    	BooleanInput buttonC = controlBinding.addBoolean("Claw deactivation");
    	//Flywheel
    	BooleanInput flywheelButton = controlBinding.addBoolean("Flywheel");
    	//Activator
    	BooleanInput actuatorButton = controlBinding.addBoolean("Shooter Trigger");
    	
    	//Turning the robot
    	BooleanInput turnToggleA = controlBinding.addBoolean("Copilot Turning Toggle 1");
    	BooleanInput turnToggleB = controlBinding.addBoolean("Copilot Turning Toggle 2");
    	BooleanInput turnToggle = turnToggleA.or(turnToggleB);
    	FloatInput copilotTurnR = controlBinding.addFloat("Turning Right").deadzone(0.2f);
    	FloatInput copilotTurnL = controlBinding.addFloat("Turning Left").deadzone(0.2f);
    	
    	//Events
    	EventOutput openClaw = () -> {
    		clawA.set(true);
    		clawB.set(false);
    	};
    	EventOutput closeClaw = () -> {
    		clawA.set(false);
    		clawB.set(true);
    	};
    	
    	EventOutput raiseArm = () -> {
    		armJoint.set(0.7f);
    	};
    	EventOutput openArm = () -> {
    		armJoint.set(0.15f);
    	};
    	EventOutput closeArm = () -> {
    		armJoint.set(0f);
    	};
    	
    	BooleanCell shouldRun = new BooleanCell(false);
    	EventOutput startWind = () -> {
    		flywheel.set(0.5f);
    		shouldRun.set(true);
    	};
    	EventOutput stopWind = () -> {
    		flywheel.set(0f);
    		shouldRun.set(false);
    	};
    	EventOutput shoot = () -> {
    		BooleanCell run = new BooleanCell(shouldRun.get());
    		new InstinctModule(run)
    		{
				protected void autonomousMain() throws Throwable {
					actuator.set(0.25f);
					waitForTime(500);
					actuator.set(0f);
					run.set(false);
				}
    		};
    	};
    	EventOutput stopShoot = () -> actuator.set(0f);
    	
    	//Sending controls
    	
    	//Drive train
    	leftTrain.send(left);
    	rightTrain.send(right);
    	turnToggle.toFloat(1f, 0f).multipliedBy(backwards).send(drive);
    	turnToggle.toFloat(-1f, 0f).multipliedBy(forwards).send(drive);
    	
    	//Arm
    	arm.multipliedBy(0.5f).send(armJoint);
    	armButtonOn.onPress(openArm);
    	armButtonHigh.onPress(raiseArm);
    	buttonO.onPress(openClaw);
    	buttonC.onPress(closeClaw);
    	
    	//Turning the robot - copilot
    	turnToggle.toFloat(0f, -0.4f).multipliedBy(copilotTurnR.minus(copilotTurnL)).send(left);
    	turnToggle.toFloat(0f, 0.4f).multipliedBy(copilotTurnR.minus(copilotTurnL)).send(right);
    	
    	//Shooting
    	flywheelButton.onPress().send(startWind);
    	flywheelButton.onRelease().send(stopWind);
    	actuatorButton.onPress().send(shoot);
    	actuatorButton.onRelease().send(stopShoot);
    	
    	//Autonomous testing - will finish sometime
    	FRC.registerAutonomous(new InstinctModule()
    	{
    	    
    	    protected void autonomousMain() throws Throwable
    	    {
    	        
    	    	clawA.set(true);
    	    	clawB.set(false);
    	    	armJoint.set(-0.9f);
    	    	waitForTime(450);
    	    	armJoint.set(-0.1f);
    	    	waitForTime(400);
    	    	armJoint.set(0f);
    	    	waitForTime(1000);
    	    	armJoint.set(0.5f);
    	    	waitForTime(1000);
    	    	armJoint.set(-0.2f);
    	    	waitForTime(666);
    	    	armJoint.set(0f);
    	    	clawA.set(false);
    	    	clawB.set(true);
    	    	armJoint.set(0.7f);
    	    	waitForTime(1000);
    	    	armJoint.set(0.12f);
    	    	
    	    	turnLeft.set(0.25f);
    	    	waitForTime(400);
    	    	turnLeft.set(0f);
    	    	waitForTime(5000);
    	    	/*
    	    	drive.set(0.4f);
    	    	waitForTime(1000);
    	    	drive.set(0f);
    	    	*/
    	    	armJoint.set(-0.2f);
    	    	waitForTime(600);
    	    	armJoint.set(0f);
    	    	clawA.set(true);
    	    	clawB.set(false);
    	    	armJoint.set(0.7f);
    	    	waitForTime(1000);
    	    	armJoint.set(0.12f);
    	    	/*
    	    	drive.set(-0.2f);
    	    	waitForTime(500);
    	    	drive.set(0f);
    	    	*/
    	    	turnLeft.set(0.25f);
    	    	waitForTime(200);
    	    	turnLeft.set(0);
    	    	/*
    	    	left.set(0.4f);
    	    	right.set(0.3f);
    	    	waitForTime(1000);
    	    	left.set(0f);
    	    	right.set(0f);
    	    	*/
    	        //Actual code:
    	        
    	        /*
    	         * NOTE: 14 inches wheel to center of robot radius
    	         * Start by facing towards a bunny, so that turning 180 degrees and moving forwards about 14 feet, then dropping the bunny will make it in the burrow
    	         * 
    	         * arm down
    	         * close claw
    	         * arm up
    	         * turn left 180 degrees
    	         * move forwards 13.5 feet
    	         * arm down
    	         * open claw
    	         * arm up
    	         * move backwards 3.5 feet
    	         * turn left 90 degrees
    	         * move in a 180 degrees arc around bin crossing lines
    	         * 
    	         */
    	        
    	    }
    	    
    	}
    	
    	);
		
	}
}
