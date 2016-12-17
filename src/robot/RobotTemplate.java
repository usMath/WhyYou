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

		Logger.info("You v0.50 2016-12-16");
		
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
		FloatOutput armJoint = FRC.talon(2).negate();
		BooleanOutput clawA = FRC.solenoid(4);
		BooleanOutput clawB = FRC.solenoid(5);
		
		//Controllers
		
		//Driver - Movement
		
		//Left side
    	FloatInput leftYJoystick1 = controlBinding.addFloat("Left drive train").deadzone(0.2f);
    	//Right side
    	FloatInput rightYJoystick1 = controlBinding.addFloat("Right drive train").deadzone(0.2f);
    	
    	//Copilot - Arm and Shooting
    	
    	//Arm
    	FloatInput leftYJoystick2 = controlBinding.addFloat("Arm").deadzone(0.2f);
    	BooleanInput armButtonOn = controlBinding.addBoolean("Arm activation");
    	BooleanInput armButtonOff = controlBinding.addBoolean("Arm deactivation");
    	//Claw
    	BooleanInput buttonO = controlBinding.addBoolean("Claw activation");
    	BooleanInput buttonC = controlBinding.addBoolean("Claw deactivation");
    	//Flywheel
    	BooleanInput flywheelButton = controlBinding.addBoolean("Flywheel");
    	//Activator
    	BooleanInput actuatorButton = controlBinding.addBoolean("Shooter Trigger");
    	
    	//Turning the robot
    	BooleanInput leftJoystickButton2 = FRC.joystick2.button(9);
    	FloatInput leftXJoystick2 = FRC.joystick2.axis(1).deadzone(0.2f);
    	//For activating control binding
    	/*
    	BooleanInput leftJoystickButton2 = controlBinding.addBoolean("Turning activation");
    	FloatInput leftXJoystick2 = controlBinding.addFloat("Copilot turning").deadzone(0.2f);
    	*/
    	
    	//Events
    	EventOutput openClaw = () -> {
    		clawA.set(true);
    		clawB.set(false);
    	};
    	EventOutput closeClaw = () -> {
    		clawA.set(false);
    		clawB.set(true);
    	};
    	
    	EventOutput openArm = () -> {
    		armJoint.set(0.05f);
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
    	leftYJoystick1.send(left);
    	rightYJoystick1.send(right);
    	
    	//Arm
    	leftYJoystick2.send(armJoint);
    	armButtonOn.onPress(openArm);
    	armButtonOff.onPress(closeArm);
    	buttonO.onPress(openClaw);
    	buttonC.onPress(closeClaw);
    	
    	//Turning the robot - copilot
    	leftJoystickButton2.toFloat(0f, -0.4f).multipliedBy(leftXJoystick2).send(left);
    	leftJoystickButton2.toFloat(0f, 0.4f).multipliedBy(leftXJoystick2).send(right);
    	
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
    	    	armJoint.set(0.2f);
    	    	waitForTime(300);
    	    	armJoint.set(0f);
    	    	clawA.set(false);
    	    	clawB.set(true);
    	    	/*
    	    	drive.set(0.4f);
    	        waitForTime(1000);
    	        drive.set(0f);
    	        waitForTime(250);
    	        drive.set(-0.4f);
    	        waitForTime(1000);
    	        drive.set(0f);
    	        waitForTime(250);
    	        drive.set(0.4f);
    	        waitForTime(1000);
    	        drive.set(0f);
    	        waitForTime(250);
    	        turnLeft.set(0.75f);
    	        waitForTime(10000);
    	        drive.set(0f);
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
