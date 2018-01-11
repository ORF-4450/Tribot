package Team4450.Tribot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

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
	  public static WPI_TalonSRX	canTalonDR1, canTalonST1, canTalonDR2;
	  public static WPI_TalonSRX	canTalonST2, canTalonDR3, canTalonST3;
	  
	  public static TribotDrive	robotDrive;

	  public final static Joystick      utilityStick = new Joystick(2);	
	  public final static Joystick      leftStick = new Joystick(0);	
	  public final static Joystick		rightStick = new Joystick(1);	
	  public final static Joystick		launchPad = new Joystick(3);

	  //public final static Compressor	compressor = new Compressor(0);	// Compressor class represents the PCM. There are 2.
	  
	  public final static AnalogInput	pressureSensor = new AnalogInput(0);
	  
	  public final static AbsoluteEncoder	encoder1 = new AbsoluteEncoder(new AnalogInput(1), 214);
	  public final static AbsoluteEncoder	encoder2 = new AbsoluteEncoder(new AnalogInput(2), 236);
	  public final static AbsoluteEncoder	encoder3 = new AbsoluteEncoder(new AnalogInput(3), 305);

	  public final static PowerDistributionPanel	PDP = new PowerDistributionPanel();

	  public final static DriverStation				ds = DriverStation.getInstance();

	  public static NavX				navx;

	  // Create RobotDrive object for CAN Talon controllers.
	  
	  public static void InitializeCANTalonDrive()
	  {
		  Util.consoleLog();

		  canTalonDR1 = new WPI_TalonSRX(1);	// front drive motor
		  canTalonST1 = new WPI_TalonSRX(2);	
		  canTalonDR2 = new WPI_TalonSRX(3);	// left drive motor looking from front
		  canTalonST2 = new WPI_TalonSRX(4);
		  canTalonDR3 = new WPI_TalonSRX(5);	// right drive motor looking from front
		  canTalonST3 = new WPI_TalonSRX(6);

		  robotDrive = new TribotDrive( canTalonDR1, canTalonDR2, canTalonDR3,
				  						canTalonST1, canTalonST2, canTalonST3,
				  						encoder1, encoder2, encoder3);

	      // Initialize CAN Talons and write status to log so we can verify
	      // all the talons are connected.
	      InitializeCANTalon(canTalonDR1);
	      InitializeCANTalon(canTalonST1);
	      InitializeCANTalon(canTalonDR2);
	      InitializeCANTalon(canTalonST2);
	      InitializeCANTalon(canTalonDR3);
	      InitializeCANTalon(canTalonST3);
      
	      // Turn on brake mode for CAN Talons.
	      SetCANTalonBrakeMode(canTalonDR1, true);
	      SetCANTalonBrakeMode(canTalonST1, true);
	      SetCANTalonBrakeMode(canTalonDR2, true);
	      SetCANTalonBrakeMode(canTalonST2, true);
	      SetCANTalonBrakeMode(canTalonDR3, true);
	      SetCANTalonBrakeMode(canTalonST3, true);
	  }

	  // Initialize and Log status indication from CANTalon. If we see an exception
	  // or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	  public static void InitializeCANTalon(WPI_TalonSRX talon)
	  {
		  Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		  talon.clearStickyFaults(0);	//0ms means no blocking.
		  //talon.enableControl();
		  //talon.changeControlMode(TalonControlMode.PercentVbus);	//TODO Find PercentVbus
	  }
	  
	  // Set neutral behavior of CAN Talons. True = brake mode, false = coast mode.

	  public static void SetCANTalonBrakeMode(WPI_TalonSRX talon, boolean brakeMode)
	  {
		  NeutralMode newMode;

		  Util.consoleLog("talon: %s  brakes on=%b", talon.getDescription(), brakeMode);
		  
		  if (brakeMode) 
			  newMode = NeutralMode.Brake;
		  else
			  newMode = NeutralMode.Coast;
		  
		  canTalonDR1.setNeutralMode(newMode);
		  canTalonST1.setNeutralMode(newMode);
		  canTalonDR2.setNeutralMode(newMode);
		  canTalonST2.setNeutralMode(newMode);
		  canTalonDR3.setNeutralMode(newMode);
		  canTalonST3.setNeutralMode(newMode);
	  }
	  
	  // Set CAN Talon voltage ramp rate. Rate is volts/sec and can be 2-12v.
	  // No replacement in 2018 WpiLib.
	  
//	  public static void SetCANTalonRampRate(WPI_TalonSRX talon, double rate)
//	  {
//		  Util.consoleLog("talon: %s  rate=%f", talon.getDescription(), rate);
//		  
//		  talon.setVoltageRampRate(rate);
//	  }
	  
	  // Return voltage and current draw for CAN Talon.
	  
	  public static String GetCANTalonStatus(WPI_TalonSRX talon)
	  {
		  return String.format("talon: %s  v=%.1f/%.1f  c=%.1f/%.1f", 
				  talon.getMotorOutputVoltage(), talon.getOutputCurrent());
	  }
}
