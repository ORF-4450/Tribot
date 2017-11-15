package Team4450.Robot11;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import Team4450.Lib.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Devices
{
	  // Motor CAN ID/PWM port assignments.
	  private static CANTalon	canTalon1, canTalon2, canTalon3;
	  private static CANTalon	canTalon4, canTalon5, canTalon6;
	  
	  public static TribotDrive	robotDrive;

	  public final static Joystick      utilityStick = new Joystick(2);	
	  public final static Joystick      leftStick = new Joystick(0);	
	  public final static Joystick		rightStick = new Joystick(1);	
	  public final static Joystick		launchPad = new Joystick(3);

	  public final static Compressor	compressor = new Compressor(0);	// Compressor class represents the PCM. There are 2.
	  
	  public final static AnalogInput	pressureSensor = new AnalogInput(0);
	  
	  public final static AbsoluteEncoder	encoder1 = new AbsoluteEncoder(new AnalogInput(1), 0);
	  public final static AbsoluteEncoder	encoder2 = new AbsoluteEncoder(new AnalogInput(2), 0);
	  public final static AbsoluteEncoder	encoder3 = new AbsoluteEncoder(new AnalogInput(3), 0);

	  public final static PowerDistributionPanel	PDP = new PowerDistributionPanel();

	  public final static DriverStation				ds = DriverStation.getInstance();

	  public static NavX				navx;

	  // Create RobotDrive object for CAN Talon controllers.
	  
	  public static void InitializeCANTalonDrive()
	  {
		  Util.consoleLog();

		  canTalon1 = new CANTalon(1);
		  canTalon2 = new CANTalon(2);
		  canTalon3 = new CANTalon(3);
		  canTalon4 = new CANTalon(4);
		  canTalon5 = new CANTalon(5);
		  canTalon6 = new CANTalon(6);

		  robotDrive = new TribotDrive( canTalon1, canTalon2, canTalon3,
				  						canTalon4, canTalon5, canTalon6,
				  						encoder1, encoder2, encoder3);

	      // Initialize CAN Talons and write status to log so we can verify
	      // all the talons are connected.
	      InitializeCANTalon(canTalon1);
	      InitializeCANTalon(canTalon2);
	      InitializeCANTalon(canTalon3);
	      InitializeCANTalon(canTalon4);
	      InitializeCANTalon(canTalon5);
	      InitializeCANTalon(canTalon6);
      
	      // Turn on brake mode for CAN Talons.
	      SetCANTalonBrakeMode(true);
	  }

	  // Initialize and Log status indication from CANTalon. If we see an exception
	  // or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	  public static void InitializeCANTalon(CANTalon talon)
	  {
		  Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		  talon.clearStickyFaults();
		  talon.enableControl();
		  talon.changeControlMode(TalonControlMode.PercentVbus);
	  }
	  
	  // Set neutral behavior of CAN Talons. True = brake mode, false = coast mode.

	  public static void SetCANTalonBrakeMode(boolean brakeMode)
	  {
		  Util.consoleLog("brakes on=%b", brakeMode);
		  
		  canTalon1.enableBrakeMode(brakeMode);
		  canTalon2.enableBrakeMode(brakeMode);
		  canTalon3.enableBrakeMode(brakeMode);
		  canTalon4.enableBrakeMode(brakeMode);
		  canTalon5.enableBrakeMode(brakeMode);
		  canTalon6.enableBrakeMode(brakeMode);
	  }
	  
	  // Set CAN Talon voltage ramp rate. Rate is volts/sec and can be 2-12v.
	  
	  public static void SetCANTalonRampRate(double rate)
	  {
		  Util.consoleLog("%f", rate);
		  
		  canTalon1.setVoltageRampRate(rate);
		  canTalon2.setVoltageRampRate(rate);
		  canTalon3.setVoltageRampRate(rate);
		  canTalon4.setVoltageRampRate(rate);
		  canTalon5.setVoltageRampRate(rate);
		  canTalon6.setVoltageRampRate(rate);
	  }
	  
	  // Return voltage and current draw for each CAN Talon.
	  
	  public static String GetCANTalonStatus()
	  {
		  return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				  canTalon1.getOutputVoltage(), canTalon1.getOutputCurrent(),
				  canTalon2.getOutputVoltage(), canTalon2.getOutputCurrent(),
				  canTalon3.getOutputVoltage(), canTalon3.getOutputCurrent(),
				  canTalon4.getOutputVoltage(), canTalon4.getOutputCurrent(),
				  canTalon5.getOutputVoltage(), canTalon5.getOutputCurrent(),
				  canTalon6.getOutputVoltage(), canTalon6.getOutputCurrent());
	  }
}
