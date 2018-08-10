package Team4450.Tribot;

import com.ctre.phoenix.motorcontrol.can.*;

import Team4450.Lib.*;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Talon;

public class TribotDrive
{
//	private final WPI_TalonSRX		driveMotor1, driveMotor2, driveMotor3;
//	private final WPI_TalonSRX		rotateMotor1, rotateMotor2, rotateMotor3;
	private final WPI_TalonSRX		driveMotor2, driveMotor3;
	private final WPI_TalonSRX		rotateMotor2, rotateMotor3;
	private final Talon				driveMotor1, rotateMotor1;
	private final AbsoluteEncoder	encoder1, encoder2, encoder3;
	private final PIDController		rotatePID1, rotatePID2, rotatePID3;
	private PIDOutputShim			rotateMotor1Shim = null, rotateMotor2Shim = null;
	private PIDSourceShim			encoder1Shim = null, encoder2Shim = null;

	// Constructor.

//	TribotDrive(WPI_TalonSRX driveMotor1, WPI_TalonSRX driveMotor2, WPI_TalonSRX driveMotor3,
//				WPI_TalonSRX rotateMotor1, WPI_TalonSRX rotateMotor2, WPI_TalonSRX rotateMotor3,
//				AbsoluteEncoder encoder1, AbsoluteEncoder encoder2, AbsoluteEncoder encoder3)
	TribotDrive(Talon driveMotor1, WPI_TalonSRX driveMotor2, WPI_TalonSRX driveMotor3,
			Talon rotateMotor1, WPI_TalonSRX rotateMotor2, WPI_TalonSRX rotateMotor3,
			AbsoluteEncoder encoder1, AbsoluteEncoder encoder2, AbsoluteEncoder encoder3)
	{
		Util.consoleLog();

		this.driveMotor1 = driveMotor1;
		this.driveMotor2 = driveMotor2;
		this.driveMotor3 = driveMotor3;
		
		this.rotateMotor1 = rotateMotor1;
		this.rotateMotor2 = rotateMotor2;
		this.rotateMotor3 = rotateMotor3;
		
		this.encoder1 = encoder1;
		this.encoder2 = encoder2;
		this.encoder3 = encoder3;
		
		//rotateMotor1Shim = new PIDOutputShim(this.rotateMotor1);
		//rotateMotor1Shim.disableOutput(true);
		
		//encoder1Shim  = new PIDSourceShim(this.encoder1);
		
		rotatePID1 = new PIDController(0.025, 0.001, 0.00, this.encoder1, this.rotateMotor1);
		configureRotationPID(rotatePID1);
		
		//encoder1Shim.setPidController(rotatePID1);

		//rotateMotor2Shim = new PIDOutputShim(this.rotateMotor2);
		//rotateMotor2Shim.disableOutput(true);
		
		//encoder2Shim  = new PIDSourceShim(this.encoder2);
		
		rotatePID2 = new PIDController(0.025, 0.001, 0.00, this.encoder2, this.rotateMotor2);
		configureRotationPID(rotatePID2);
		
		//encoder2Shim.setPidController(rotatePID2);
		
		rotatePID3 = new PIDController(0.025, 0.001, 0.00, this.encoder3, this.rotateMotor3);
		configureRotationPID(rotatePID3);
	}

	/**
	 *  Free all objects that need it.
	 */
	void dispose()
	{
		Util.consoleLog();

		rotatePID1.disable();
		rotatePID1.free();
		rotatePID2.disable();
		rotatePID2.free();
		rotatePID3.disable();
		rotatePID3.free();
	}
	
	/**
	 * Set common PID values for motor module rotation control.
	 * @param pid PID controller to configure.
	 */
	private void configureRotationPID(PIDController pid)
	{
		pid.setAbsoluteTolerance(1);	// degrees.
		pid.setOutputRange(-1.0, 1.0);
		pid.setContinuous(false);
	}
	
	/**
	 * Turn all motors to their zero angle to get all 3 motors aligned in the same
	 * direction, which should be to the front of the robot.
	 */
	void alignToZero()
	{
		Util.consoleLog();

		// First alignment done with encoders in absolute angle mode.
		
		pointMotor(rotatePID1, 0);
		pointMotor(rotatePID2, 0);
		pointMotor(rotatePID3, 0);
		
		// Driving done with encoders in offset from zero mode.
		
		this.encoder1.setPidOffsetMode(true);
		this.encoder2.setPidOffsetMode(true);
		this.encoder3.setPidOffsetMode(true);
	}
	
	/**
	 * Turn motor assembly to desired angle. PID controller does the work
	 * on it's own thread reading encoder and setting motor power to turn to the
	 * target angle.
	 * @param angle Desired rotational angle of motor from encoder zero point. Depending
	 * on encoder mode, this is 0-360 or -180 to +180.
	 */
	private void pointMotor(PIDController pid, int angle)
	{
		//Util.consoleLog("angle=%d", angle);
		
		pid.setSetpoint(angle);
		pid.enable();
	}

	/**
	 * Drive robot with wheel #1 only with specified power and angle value.
	 * @param power Motor power -1 to +1.
	 * @param angle Steering angle -1 to +1 which is normalized to -90 to +90.
	 */
	void singleSteer(double power, double angle)
	{
		//Util.consoleLog("angle=%f", angle);

		angle = angle * 90;
		
		pointMotor(rotatePID1, (int) angle);
		
		driveMotor1.set(power);
		driveMotor2.set(power);
		driveMotor3.set(power);
	}

	/**
	 * Drive robot with all wheels with specified power and angle value.
	 * @param power Motor power -1 to +1.
	 * @param angle Steering angle -1 to +1 which is normalized to -90 to +90.
	 */
	void allSteer(double power, double angle)
	{
		//Util.consoleLog("angle=%f", angle);

		angle = angle * 90;
		
		pointMotor(rotatePID1, (int) angle);
		pointMotor(rotatePID2, (int) angle);
		pointMotor(rotatePID3, (int) angle);

		driveMotor1.set(power);
		driveMotor2.set(power);
		driveMotor3.set(power);
	}
	
	/**
	 * Set drive to rotate mode. This means turn all 3 wheels to align so
	 * that the bot rotates in place when power is applied.
	 * @param rotate True sets wheels to rotate position, false sets wheels
	 * to zero position (straight ahead).
	 */
	void setRotateMode(boolean rotate)
	{
		if (rotate)
		{
			// 1 = -86  2= 150  3 = 30 (degrees from zero)
			pointMotor(rotatePID1, -86);
			pointMotor(rotatePID2, 150);
			pointMotor(rotatePID3, 30);
		}
		else
		{
			pointMotor(rotatePID1, 0);
			pointMotor(rotatePID2, 0);
			pointMotor(rotatePID3, 0);
		}
		
	}
	
	/**
	 * Set motor power when in rotate mode.
	 * @param power Power level -1 (left) to +1 (right).
	 */
	void SetRotatePower(double power)
	{
		driveMotor1.set(power);
		driveMotor2.set(power);
		driveMotor3.set(power);
	}

	/**
	 * Enable/disable motor safety.
	 * @param enabled True to enable motor safety protection.
	 */
	void setSafetyEnabled(boolean enabled)
	{
		Util.consoleLog("%b", enabled);
		
		driveMotor1.setSafetyEnabled(enabled);
		driveMotor2.setSafetyEnabled(enabled);
		driveMotor3.setSafetyEnabled(enabled);
		
		rotateMotor1.setSafetyEnabled(enabled);
		rotateMotor2.setSafetyEnabled(enabled);
		rotateMotor3.setSafetyEnabled(enabled);
	}
	
	/**
	 * Set timeout for motor safety.
	 * @param timeout seconds to wait for motor input before disabling motors.
	 */
	void setExpiration(double timeout)
	{
		Util.consoleLog("%f", timeout);
		
		driveMotor1.setExpiration(timeout);
		driveMotor2.setExpiration(timeout);
		driveMotor3.setExpiration(timeout);
		
		rotateMotor1.setExpiration(timeout);
		rotateMotor2.setExpiration(timeout);
		rotateMotor3.setExpiration(timeout);
	}
	
	/**
	 * Stop the all motors on the drive.
	 */
	void stopMotors()
	{
		driveMotor1.set(0);
		driveMotor2.set(0);
		driveMotor3.set(0);
		
		rotatePID1.disable();
		rotateMotor1.set(0);
		rotatePID2.disable();
		rotateMotor2.set(0);
		rotatePID3.disable(); 
		rotateMotor3.set(0);
	}
	
	/**
	 * Invert drive motor control signals.
	 * @param motor Motor to set inversion status.
	 * 1 = motor 1, 2 = motor 2, 3 = motor 3.
	 * @param isInverted True to invert motor control.
	 */
	void setInvertedMotor(int motor, boolean isInverted)
	{
		Util.consoleLog("%d inverted=%b", motor, isInverted);
		
		switch (motor)
		{
			case 1:
				driveMotor1.setInverted(isInverted);
			
			case 2:
				driveMotor2.setInverted(isInverted);
				
			case 3:
				driveMotor3.setInverted(isInverted);
		}
	}
}
