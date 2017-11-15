package Team4450.Robot11;

import com.ctre.CANTalon;

import Team4450.Lib.*;
import edu.wpi.first.wpilibj.PIDController;

public class TribotDrive
{
	private final CANTalon			driveMotor1, driveMotor2, driveMotor3;
	private final CANTalon			rotateMotor1, rotateMotor2, rotateMotor3;
	private final AbsoluteEncoder	encoder1, encoder2, encoder3;
	private final PIDController		rotatePID1, rotatePID2, rotatePID3;

	// Constructor.

	TribotDrive(CANTalon driveMotor1, CANTalon driveMotor2, CANTalon driveMotor3,
				CANTalon rotateMotor1, CANTalon rotateMotor2, CANTalon rotateMotor3,
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
		
		rotatePID1 = new PIDController(0.0, 0.0, 0.0, this.encoder1, this.rotateMotor1);
		configureRotationPID(rotatePID1);
		rotatePID2 = new PIDController(0.0, 0.0, 0.0, this.encoder2, this.rotateMotor2);
		configureRotationPID(rotatePID2);
		rotatePID3 = new PIDController(0.0, 0.0, 0.0, this.encoder3, this.rotateMotor3);
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
		pid.enable();
	}
	
	/**
	 * Turn all motors to their zero angle to get all 3 motors aligned in the same
	 * direction, which should be to the front of the robot.
	 */
	void alignToZero()
	{
		Util.consoleLog();

		pointMotor(rotatePID1, 0);
		pointMotor(rotatePID2, 0);
		pointMotor(rotatePID3, 0);
	}
	
	/**
	 * Turn motor assembly to encoder position. PID controller does the work
	 * on it's own thread reading encoder and setting motor power to turn to the
	 * target angle.
	 * @param position Desired rotational angle of motor 0-360.
	 */
	private void pointMotor(PIDController pid, int angle)
	{
		Util.consoleLog("pos=%d", angle);
		
		pid.setSetpoint(angle);
	}

	/**
	 * Drive robot with specified power and angle value.
	 * @param power Motor power -1 to +1.
	 * @param angle Steering angle -1 to +1 which is normalized to -90 to +90.
	 */
	void singleSteer(double power, double angle)
	{
		// to steer, we have to convert the joystick input to the range -90  to +90
		// and then add that to the zero angle. So zero input angle = the motor zero
		// angle, meaning no joystick deflection motor points straight ahead.
		
		pointMotor(rotatePID1, (int) (angle * 90));
		
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
	void stopMotor()
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
	 * Invert motor control signals.
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
