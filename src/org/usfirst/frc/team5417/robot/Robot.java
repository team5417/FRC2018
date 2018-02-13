/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5417.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import org.usfirst.frc.team5417.robot.XBoxController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot implements PIDSource, PIDOutput {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
//	private static final String LeftLeft = "Left Position Left Switch";
//	private static final String LeftLeft = "Left Position Left Switch";
//	private static final String LeftRight = "Left Position Right Switch";
//	private static final String LeftRight = "Left Position Right Switch";
//	private static final String CenterLeft = "Center Position Left Switch";
//	private static final String CenterLeft = "Center Position Left Switch";
//	private static final String CenterRight = "Center Position Right Switch";
//	private static final String CenterRight = "Center Position Right Switch";
//	private static final String RightLeft = "Right Position Left Switch";
//	private static final String RightLeft = "Right Position Left Switch";
//	private static final String RightRight = "Right Position Right Switch";
//	private static final String RightRight = "Right Position Right Switch";
	private String m_autoSelected;
	
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(1);
	WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(2);
	WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(4);
	WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(5);
	WPI_TalonSRX armMotor1 = new WPI_TalonSRX(3);
	WPI_TalonSRX armMotor2 = new WPI_TalonSRX(6);
	Compressor compressor = new Compressor();
	Solenoid gearShift = new Solenoid(0);
	Timer timer = new Timer();
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor1, leftMotor2);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor1, rightMotor2);
	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
	XBoxController driverStick = new XBoxController(new Joystick(0));
	XBoxController manipulatorStick = new XBoxController(new Joystick(1));
	int leftPosition = 0;
	int rightPosition = 0;
	boolean buttonstatus = false;
	//NavxPID navxPID = new NavxPID(.1,20000, 0);
	PIDController navxPID = new PIDController(.03, 0, .1, this, this);
	double left;
	double right;
	AHRS navx;
	float yaw = 0 ;
	final String defaultAuto = "Default Autonomous";
	final String gearTestAuto = "Left auto";
	final String otherAuto = "other auto";
	final String centerAuto = "Center auto";
	final String driveStraight = "Drive straight";
	final String rightAuto = "Right auto";
	double multiplier = 260.759845021053;
//	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, .05, .1, .5, .1);
//	Waypoint[] points = new Waypoint[] {
//		new Waypoint(0,0, Pathfinder.d2r(90)),
//		new Waypoint(0,3,Pathfinder.d2r(90)),
//		//new Waypoint(2,7,Pathfinder.d2r(45))
//	};
//	Trajectory trajectory = Pathfinder.generate(points, config);
//	TankModifier modifier = new TankModifier(trajectory).modify(.66);
//	Trajectory leftTrajectory = modifier.getLeftTrajectory();
//	Trajectory rightTrajectory = modifier.getRightTrajectory();
//	EncoderFollower leftFollower = new EncoderFollower(leftTrajectory);
//	EncoderFollower rightFollower = new EncoderFollower(rightTrajectory);
	double leftSpeed = 0;
	double rightSpeed = 0;
	int initLeftPosition;
	int initRightPosition;
	double correction = 0;
	double initialYaw;
	double currentYaw;
	double targetYaw;
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		navx = new AHRS(SerialPort.Port.kUSB);
		navx.reset();
		leftMotor1.setInverted(true);
		leftMotor2.setInverted(true);
		rightMotor1.setInverted(true);
		rightMotor2.setInverted(true);
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		CameraServer server = CameraServer.getInstance();
		server.startAutomaticCapture("cam0", 0);
		navxPID.setOutputRange(-0.5,.5);
		navxPID.setAbsoluteTolerance(.01);
		navxPID.setInputRange(-180, 180);
		navxPID.enable();
		initialYaw = navx.getYaw();
		m_autoSelected = kDefaultAuto ;
//		leftFollower.configurePIDVA(.8, 0, 0, 1/1.7, 0);
//		rightFollower.configurePIDVA(.8, 0, 0, 1/1.7, 0);
//		rightFollower.configureEncoder(0, 4096, .127);
//		leftFollower.configureEncoder(0, 4096, .127);
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		//m_autoSelected = m_chooseetSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector",
		// 		kDefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		gearShift.set(false);
		initLeftPosition = leftMotor1.getSelectedSensorPosition(0);
		initRightPosition = leftMotor1.getSelectedSensorPosition(0);
	}
	

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				
				
				break;
			case kDefaultAuto:
//				gearShift.set(false);
//				initLeftPosition = leftMotor1.getSelectedSensorPosition(0);
//				initRightPosition = leftMotor1.getSelectedSensorPosition(0);
//				while (!leftFollower.isFinished() & !rightFollower.isFinished()) {
//					
//					leftPosition = leftMotor1.getSelectedSensorPosition(0) - initLeftPosition;
//					rightPosition = rightMotor2.getSelectedSensorPosition(0)- initRightPosition;
//					leftSpeed = leftFollower.calculate(leftPosition);
//					rightSpeed = rightFollower.calculate(rightPosition);
//					m_drive.tankDrive(leftSpeed, rightSpeed);
//					//Trajectory.Segment seg = leftFollower.getSegment();
//					//SmartDashboard.putString("DB/String 1", seg.x + ", " + seg.y);
//					SmartDashboard.putString("DB/String 5", "Left adjusted: " +leftPosition);
//					SmartDashboard.putString("DB/String 6", "Right adjusted: " +rightPosition);
//					
//					SmartDashboard.putString("DB/String 0", "Auton Finished: false");
//				}
//				
//				SmartDashboard.putString("DB/String 0", "Auton Finished: true");
				break;
			default:
				navx.zeroYaw();
				while(yaw < 70 ) {
					yaw = navx.getYaw();
					m_drive.tankDrive(.6, -.6);
					SmartDashboard.putString("DB/String 4", "Yaw: " + yaw);
					SmartDashboard.putString("DB/String 5", "Checking navx");
					SmartDashboard.putString("DB/String 6", "left: " + m_left.get());
					SmartDashboard.putString("DB/String 7", "Right: " + m_right.get());
				}
				while(yaw < 90 ) {
					yaw = navx.getYaw();
					m_drive.tankDrive(.5, -.5);
					SmartDashboard.putString("DB/String 4", "Yaw: " + yaw);
					SmartDashboard.putString("DB/String 5", "Checking navx");
					SmartDashboard.putString("DB/String 6", "left: " + m_left.get());
					SmartDashboard.putString("DB/String 7", "Right: " + m_right.get());
				}
				m_drive.tankDrive(0, 0);
				SmartDashboard.putString("DB/String 5", "Stopped robot");
				break;
			
		}
	}

	@Override
	public void teleopInit() {
		compressor.start();
		timer.start();
		//Arm.setSetpoint(motor.getSelectedSensorPosition(0));
	}
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		SmartDashboard.putNumber("left", left);
		SmartDashboard.putNumber("right", right);
		
		rightPosition = rightMotor2.getSelectedSensorPosition(0);
		SmartDashboard.putString("DB/String 2", "Right Position: " + rightPosition);
		leftPosition = leftMotor1.getSelectedSensorPosition(0);
		SmartDashboard.putString("DB/String 0", "Left Position: " + leftPosition);
		
		float yaw = navx.getYaw();
		SmartDashboard.putString("DB/String 4", "Z axis: " + yaw);
		SmartDashboard.putString("DB/String 5", "" + compressor.enabled());
		if (driverStick.isFirstLBPressed()) {
			gearShift.set(true);	
		}
		if (driverStick.isFirstRBPressed()) {
			gearShift.set(false);
		}

		if (driverStick.isFirstYPressed()) {
			double currentYaw = navx.getYaw();
			navxPID.setSetpoint(currentYaw);
		}
		if (driverStick.isFirstXPressed()) {
			timer.reset();
		}
		if (driverStick.isXHeldDown()) {
			armMotor1.set(0);
			armMotor2.set(0);
		}	
		if (driverStick.isBHeldDown()) {
			armMotor1.set(0);
			armMotor2.set(0);
		}
		
		if (driverStick.isYHeldDown()) {
			SmartDashboard.putNumber("c",  correction);
			double speedMultiplier = getMultiplier(timer.get(), 1.5);
			left = right = .8 * speedMultiplier;
			left = left - correction;
			right = right + correction;
			m_drive.tankDrive(left, right);
			SmartDashboard.putNumber("l", (leftSpeed - correction));
			SmartDashboard.putNumber("r", (rightSpeed + correction));
		}
		else if (driverStick.isXHeldDown()) {
			/*double speedMultiplier = getMultiplier(timer.get(), 3);
			left = right = .8 * speedMultiplier;
			m_drive.tankDrive(left,right);
			SmartDashboard.putNumber("l", left);
			SmartDashboard.putNumber("r", right);
			*/
			initialYaw = navx.getYaw();
			currentYaw = initialYaw;
			targetYaw = initialYaw+90;
			while(currentYaw<targetYaw) {
				m_drive.tankDrive(-.5, .5);
				currentYaw=navx.getYaw();
			}
		}
		else {
			left = -driverStick.getLYValue();
			right = -driverStick.getRYValue();
			m_drive.tankDrive(left,right);
		}

	}
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	@Override
	public void pidWrite(double output) {
		SmartDashboard.putString("DB/String 6", "PID Output: " + output);
		correction = output;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		
		return navx.getYaw();
	}
	
	public int inchesToTicks(double inches) {
		int ticks = (int) (inches * multiplier);
		return ticks;
	}
	public double getMultiplier(double time, double maxTime) {
		double multiplier = (Math.pow(time, 2))/(Math.pow(maxTime,2));
		if (multiplier < 0) {
			return 0;
		}
		else if (multiplier > 1) {
			return 1;
		}
		else {
			return multiplier;
		}
	}
}
