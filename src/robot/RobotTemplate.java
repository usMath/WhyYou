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

		Logger.info("You v0.43 2016-12-14");
		
		//Control Binding
		final ControlBindingCreator controlBinding = FRC.controlBinding();
		
		//x234567
		// Right drive train (1, x, x)
		FloatOutput right1 = FRC.talonCAN(1).simpleControl();
		FloatOutput right2 = FRC.talonCAN(1).simpleControl();
		FloatOutput right3 = FRC.talonCAN(1).simpleControl();
		
		FloatOutput right = right1.combine(right2).combine(right3).addRamping(0.02f, FRC.constantPeriodic);
		
		// Left drive train (0, 8, 9)
		FloatOutput left1 = FRC.talonCAN(0).simpleControl();
		FloatOutput left2 = FRC.talonCAN(8).simpleControl();
		FloatOutput left3 = FRC.talonCAN(9).simpleControl();
		
		FloatOutput left = left1.combine(left2).combine(left3).addRamping(0.02f, FRC.constantPeriodic).negate();
		
		// Shooter
		FloatOutput flywheel = FRC.talon(0);
		FloatOutput actuator = FRC.talon(1);
		
		// Combined drive train
		FloatOutput drive = left.combine(right);
    	FloatOutput turnLeft = right.combine(left.negate());
    	FloatOutput turnRight = left.combine(right.negate());
		
		// Arm has lower joint, upper joint, and claw. Similar to human arm (hand = claw).
		FloatOutput armJointLower = FRC.talon(2);
		BooleanOutput claw = FRC.solenoid(1);
		
		//Controllers
		
		//Driver - Movement
		
		//Left side
    	FloatInput leftYJoystick1 = controlBinding.addFloat("Left drive train").deadzone(0.2f);
    	//Right side
    	FloatInput rightYJoystick1 = controlBinding.addFloat("Right drive train").deadzone(0.2f);
    	
    	//Copilot - Arm and Shooting
    	
    	//Arm
    	FloatInput leftYJoystick2 = controlBinding.addFloat("Lower arm").deadzone(0.2f);
    	//Claw
    	BooleanInput button2 = controlBinding.addBoolean("Claw activation");
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
    	leftYJoystick2.send(armJointLower);
    	button2.send(claw);
    	
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
