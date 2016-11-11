package robot;

import ccre.channel.BooleanCell;
import ccre.channel.BooleanInput;
import ccre.channel.BooleanOutput;
import ccre.channel.EventOutput;
import ccre.channel.FloatInput;
import ccre.channel.FloatOutput;
import ccre.ctrl.ExtendedMotor.OutputControlMode;
import ccre.ctrl.ExtendedMotorFailureException;
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

		Logger.info("You v0.1 2016-11-09");
		
		// Right drive train
		FloatOutput right1 = FRC.talonCAN(1).simpleControl();
		FloatOutput right2 = FRC.talonCAN(2).simpleControl();
		FloatOutput right3 = FRC.talonCAN(3).simpleControl();
		
		FloatOutput right = right1.combine(right2).combine(right3);
		
		// Left drive train
		FloatOutput left1 = FRC.talonCAN(4).simpleControl();
		FloatOutput left2 = FRC.talonCAN(5).simpleControl();
		FloatOutput left3 = FRC.talonCAN(6).simpleControl();
		
		FloatOutput left = left1.combine(left2).combine(left3);
		
		// Arm has lower joint, upper joint, and claw. Similar to human arm (hand = claw).
		FloatOutput armJointLower = FRC.talonCAN(7).simpleControl();
		FloatOutput armJointUpper = FRC.talonCAN(8).simpleControl();
		BooleanOutput claw = FRC.solenoid(1);
		
		//Controllers
		
		//Driver - Movement
		
		//Left side
    	FloatInput leftYJoystick1 = FRC.joystick1.axis(2).deadzone(0.2f);
    	//Right side
    	FloatInput rightYJoystick1 = FRC.joystick1.axis(6).deadzone(0.2f);
    	
    	//Copilot - Arm
    	
    	//Lower arm
    	FloatInput leftYJoystick2 = FRC.joystick2.axis(2).deadzone(0.2f);
    	//Upper arm
    	FloatInput rightYJoystick2 = FRC.joystick2.axis(6).deadzone(0.2f);
    	//Claw
    	BooleanInput leftButton2 = FRC.joystick2.button(5);
    	BooleanInput rightButton2 = FRC.joystick2.button(6);
    	BooleanInput button2 = leftButton2.or(rightButton2);
    	
    	//Turning the robot
    	FloatInput leftXJoystick2 = FRC.joystick2.axis(1).deadzone(0.2f);
    	
    	
    	
		
	}
}
