
package Team4450.Tribot;

import java.lang.Math;

import Team4450.Lib.*;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import edu.wpi.first.wpilibj.Timer;

class Teleop
{
	private final Robot 		robot;
	public  JoyStick			rightStick, leftStick, utilityStick;
	public  LaunchPad			launchPad;
	private boolean				autoTarget, altDriveMode, setEncoderZero, rotateMode;

	// Constructor.

	Teleop(Robot robot)
	{
		Util.consoleLog();

		this.robot = robot;
	}

	// Free all objects that need it.

	void dispose()
	{
		Util.consoleLog();

		if (leftStick != null) leftStick.dispose();
		if (rightStick != null) rightStick.dispose();
		if (utilityStick != null) utilityStick.dispose();
		if (launchPad != null) launchPad.dispose();
	}

	void OperatorControl() throws Exception
	{
		double	rightY = 0, rightX = 0, leftY = 0, leftX = 0, utilX = 0;
		
		// Motor safety turned off during initialization.
		Devices.robotDrive.setSafetyEnabled(false);

		Util.consoleLog();

		LCD.printLine(1, "Mode: OperatorControl");
		LCD.printLine(2, "All=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, Devices.ds.isFMSAttached());

		// Configure LaunchPad and Joystick event handlers.

		launchPad = new LaunchPad(Devices.launchPad, LaunchPadControlIDs.BUTTON_BLUE);

		LaunchPadControl lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_BACK);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;

		lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_FRONT);
		lpControl.controlType = LaunchPadControlTypes.SWITCH;

		//Example on how to track button:
		//launchPad.AddControl(LaunchPadControlIDs.BUTTON_COLOR_HERE);
		launchPad.addLaunchPadEventListener(new LaunchPadListener());
		launchPad.Start();

		leftStick = new JoyStick(Devices.leftStick, "LeftStick", JoyStickButtonIDs.TRIGGER);
		//Example on how to track button:
		leftStick.AddButton(JoyStickButtonIDs.TOP_BACK);
		leftStick.addJoyStickEventListener(new LeftStickListener());
		leftStick.Start();

		rightStick = new JoyStick(Devices.rightStick, "RightStick", JoyStickButtonIDs.TRIGGER);
		//Example on how to track button:
		//rightStick.AddButton(JoyStickButtonIDs.BUTTON_NAME_HERE);
		rightStick.addJoyStickEventListener(new RightStickListener());
		rightStick.Start();

		utilityStick = new JoyStick(Devices.utilityStick, "UtilityStick", JoyStickButtonIDs.TRIGGER);
		//Example on how to track button:
		//utilityStick.AddButton(JoyStickButtonIDs.BUTTON_NAME_HERE);
		utilityStick.addJoyStickEventListener(new UtilityStickListener());
		utilityStick.Start();

		// Remove dead zone for smoother steering movement.
		rightStick.deadZone(.03);
		leftStick.deadZone(.03);

		// Set CAN Talon brake mode by rocker switch setting.
		// We do this here so that the Utility stick thread has time to read the initial state
		// of the rocker switch.
		//if (robot.isComp) Devices.SetCANTalonBrakeMode(lpControl.latchedState);

		// Set gyro/Navx to heading 0.
		//Devices.navx.resetYaw();

		// Reset encoder.
		//Devices.encoder.reset();

		// Motor safety turned on.
		Devices.robotDrive.setSafetyEnabled(true);

		// Driving loop runs until teleop is over.

		while (robot.isEnabled() && robot.isOperatorControl())
		{
			// Get joystick deflection and feed to robot drive object
			// using calls to our JoyStick class.

			rightY = stickLogCorrection(rightStick.GetY());	// fwd/back
			leftX = leftStick.GetX();						// left/right
			rightX = rightStick.GetX();						// left/right

			utilX = utilityStick.GetX();

			LCD.printLine(4, "leftX=%.3f  rightX=%.3f  rightY=%.3f  utilX=%.3f", leftX, rightX, rightY, utilX);
			//LCD.printLine(6, "yaw=%.2f, total=%.2f, rate=%.2f, hdng=%.2f", Devices.navx.getYaw(), Devices.navx.getTotalYaw(), 
			//		Devices.navx.getYawRate(), Devices.navx.getHeading());
			LCD.printLine(5, "Set encoder zero=%b", setEncoderZero);
			LCD.printLine(6, "encoderR1=%d  encoderR2=%d  encoderR3=%d", Devices.encoder1.getRawAngle(), Devices.encoder2.getRawAngle(), Devices.encoder3.getRawAngle());
			LCD.printLine(7, "encoder1=%d  encoder2=%d  encoder3=%d", Devices.encoder1.getAngle(), Devices.encoder2.getAngle(), Devices.encoder3.getAngle());
			LCD.printLine(8, "encoder1O=%d  encoder2O=%d  encoder3O=%d", Devices.encoder1.getOffsetFromZero(), Devices.encoder2.getOffsetFromZero(), Devices.encoder3.getOffsetFromZero());
			//LCD.printLine(8, "pressureV=%.2f  psi=%d", robot.monitorCompressorThread.getVoltage(), robot.monitorCompressorThread.getPressure());

			// Set wheel motors.
			// Do not feed JS input to robotDrive if we are controlling the motors in automatic functions.

			if (!autoTarget && !setEncoderZero && !rotateMode && !altDriveMode) Devices.robotDrive.allSteer(rightY * .50, leftX);
			
			if (!autoTarget && !setEncoderZero && !rotateMode && altDriveMode) Devices.robotDrive.singleSteer(rightY * .50, leftX);

			if (!autoTarget && !setEncoderZero && rotateMode) Devices.robotDrive.SetRotatePower(leftX);

			// Update the robot heading indicator on the DS.

			//SmartDashboard.putNumber("Gyro", Devices.navx.getHeading());

			// End of driving loop.

			Timer.delay(.020);	// wait 20ms for update from driver station.
		}

		// End of teleop mode.

		Util.consoleLog("end");
	}

	// Custom base logarithm.
	// Returns logarithm base of the value.

	private double baseLog(double base, double value)
	{
		return Math.log(value) / Math.log(base);
	}

	// Map joystick y value of 0.0 to 1.0 to the motor working power range of approx 0.5 to 1.0 using
	// Logarithmic curve.

	private double stickLogCorrection(double joystickValue)
	{
		double base = Math.pow(2, 1/3) + Math.pow(2, 1/3);

		if (joystickValue > 0)
			joystickValue = baseLog(base, joystickValue + 1);
		else if (joystickValue < 0)
			joystickValue = -baseLog(base, -joystickValue + 1);

		return joystickValue;
	}

	// Handle LaunchPad control events.

	public class LaunchPadListener implements LaunchPadEventListener 
	{
		public void ButtonDown(LaunchPadEvent launchPadEvent) 
		{
			LaunchPadControl	control = launchPadEvent.control;

			Util.consoleLog("%s, latchedState=%b", control.id.name(),  control.latchedState);

			switch(control.id)
			{
			//Example of case:
			/*
			case BUTTON_NAME_HERE:
				if (launchPadEvent.control.latchedState)
					DoOneThing();
				else
					DoOtherThing();
				break;
			*/
			default:
				break;
			}
		}

		public void ButtonUp(LaunchPadEvent launchPadEvent) 
		{
			//Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.name(),  launchPadEvent.control.latchedState);
		}

		public void SwitchChange(LaunchPadEvent launchPadEvent) 
		{
			LaunchPadControl	control = launchPadEvent.control;

			Util.consoleLog("%s, latchedState=%b", control.id.name(), control.latchedState);

			switch(control.id)
			{
			// Example of Rocker:
			
			case ROCKER_LEFT_FRONT:
				if (control.latchedState)
				{
					Devices.robotDrive.setSafetyEnabled(false);
					setEncoderZero = true;
					Devices.robotDrive.stopMotors();
				}
				else
				{
					Devices.encoder1.setZeroAngleOffset(Devices.encoder1.getRawAngle());
					Devices.encoder2.setZeroAngleOffset(Devices.encoder2.getRawAngle());
					Devices.encoder3.setZeroAngleOffset(Devices.encoder3.getRawAngle());

					setEncoderZero = false;
					Devices.robotDrive.setSafetyEnabled(true);
				}
				
				break;
			
			default:
				break;
			}
		}
	}

	// Handle Right JoyStick Button events.

	private class RightStickListener implements JoyStickEventListener 
	{

		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
			case TRIGGER:
				altDriveMode = !altDriveMode;
				break;
				
			//Example of Joystick Button case:
			/*
			case TRIGGER:
				if (button.latchedState)
					DoOneThing();
				else
					DoOtherThing();
				break;
			 */
			default:
				break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.name());
		}
	}

	// Handle Left JoyStick Button events.

	private class LeftStickListener implements JoyStickEventListener 
	{
		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TOP_BACK:
					if (button.latchedState)
						rotateMode = true;
					else
						rotateMode = false;
					
					Devices.robotDrive.setRotateMode(rotateMode);
					break;
				
				//Example of Joystick Button case:
			/*
			case TRIGGER:
				if (button.latchedState)
					DoOneThing();
				else
					DoOtherThing();
				break;
			 */
			default:
				break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.name());
		}
	}

	// Handle Utility JoyStick Button events.

	private class UtilityStickListener implements JoyStickEventListener 
	{
		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
			//Example of Joystick Button case:
			/*
			case TRIGGER:
				if (button.latchedState)
					DoOneThing();
				else
					DoOtherThing();
				break;
			 */
			default:
				break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.id.name());
		}
	}
}
