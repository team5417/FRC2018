
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
import edu.wpi.first.wpilibj.DigitalInput;
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
	private String m_autoSelected;

	private SendableChooser<String> m_chooser = new SendableChooser<>();
	WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(1);
	WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(2);
	WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(5);
	WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(6);
	WPI_TalonSRX armMotor1 = new WPI_TalonSRX(0);
	WPI_TalonSRX armMotor2 = new WPI_TalonSRX(4);
	VictorSP leftIntake = new VictorSP(0);
	VictorSP rightIntake = new VictorSP(1);
	Compressor compressor = new Compressor();
	Solenoid gearShiftLeft = new Solenoid(0);
	Solenoid gearShiftRight = new Solenoid(1);
	DigitalInput limitSwitchTop = new DigitalInput(0);
	DigitalInput limitSwitchBottom = new DigitalInput(1);
	Timer timer = new Timer();
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor1, leftMotor2);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor1, rightMotor2);
	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
	XBoxController driverStick = new XBoxController(new Joystick(0));
	XBoxController manipulatorStick = new XBoxController(new Joystick(1));
	int leftPosition = 0;
	int rightPosition = 0;
	boolean buttonstatus = false;
	// NavxPID navxPID = new NavxPID(.1,20000, 0);
	PIDController navxPID = new PIDController(.01, 0, .1, this, this);
	double left;
	double right;
	AHRS navx;
	ContinuousAngleTracker cat;
	double yaw = 0;
	final String defaultAuto = "Default Autonomous";
	final String gearTestAuto = "Left auto";
	final String otherAuto = "other auto";
	final String centerAuto = "Center auto";
	final String driveStraight = "Drive straight";
	final String rightAuto = "Right auto";
	double multiplier = 0;
	// Trajectory.Config config = new
	// Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
	// Trajectory.Config.SAMPLES_FAST, .05, .1, .5, .1);
	// Waypoint[] points = new Waypoint[] {
	// new Waypoint(0,0, Pathfinder.d2r(90)),
	// new Waypoint(0,3,Pathfinder.d2r(90)),
	// //new Waypoint(2,7,Pathfinder.d2r(45))
	// };
	// Trajectory trajectory = Pathfinder.generate(points, config);
	// TankModifier modifier = new TankModifier(trajectory).modify(.66);
	// Trajectory leftTrajectory = modifier.getLeftTrajectory();
	// Trajectory rightTrajectory = modifier.getRightTrajectory();
	// EncoderFollower leftFollower = new EncoderFollower(leftTrajectory);
	// EncoderFollower rightFollower = new EncoderFollower(rightTrajectory);
	double leftSpeed = 0;
	double rightSpeed = 0;
	int initLeftPosition;
	int initRightPosition;
	double correction = 0;
	double initialYaw;
	double currentYaw;
	double targetYaw;
	boolean run = false;
	double lastYaw;
	boolean turnCompleted = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		navx = new AHRS(SerialPort.Port.kUSB);
		navx.reset();
		
		cat = new ContinuousAngleTracker();
		
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
//		navxPID.setOutputRange(-0.5, .5);
//		navxPID.setAbsoluteTolerance(.01);
//		navxPID.setInputRange(-180, 180);
////	navxPID.enable();
		resetNavxPID();
		m_autoSelected = kCustomAuto;
		// leftFollower.configurePIDVA(.8, 0, 0, 1/1.7, 0);
		// rightFollower.configurePIDVA(.8, 0, 0, 1/1.7, 0);
		// rightFollower.configureEncoder(0, 4096, .127);
		// leftFollower.configureEncoder(0, 4096, .127);

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		// m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector",
		// kDefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		gearShiftLeft.set(false);
		gearShiftRight.set(false);
		initLeftPosition = leftMotor2.getSelectedSensorPosition(0);
		initRightPosition = rightMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// update the continuous angle tracker with the current angle so it can track angles over time
		cat.nextAngle(navx.getYaw());
		
		switch (m_autoSelected) {
		case kCustomAuto:
			if (!run) {
				//driveDistance(10*12);
				//turnLeft();
				SmartDashboard.putString("DB/String 9", "Final: " + cat.getAngle());
				run = true;
			}
			break;
		case kDefaultAuto:
			//
			break;
		default:
//			navx.zeroYaw();
			while (yaw < 70) {
//				yaw = navx.getYaw();
				yaw = cat.getAngle();
				m_drive.tankDrive(.6, -.6);
				SmartDashboard.putString("DB/String 4", "Yaw: " + yaw);
//				SmartDashboard.putString("DB/String 5", "Checking navx");
				SmartDashboard.putString("DB/String 6", "left: " + m_left.get());
				SmartDashboard.putString("DB/String 7", "Right: " + m_right.get());
			}
			while (yaw < 90) {
//				yaw = navx.getYaw();
				yaw = cat.getAngle();
				m_drive.tankDrive(.5, -.5);
				SmartDashboard.putString("DB/String 4", "Yaw: " + yaw);
//				SmartDashboard.putString("DB/String 5", "Checking navx");
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
		// Arm.setSetpoint(motor.getSelectedSensorPosition(0));
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		// update the continuous angle tracker with the current angle so it can track angles over time
		cat.nextAngle(navx.getYaw());

		SmartDashboard.putNumber("left", left);
		SmartDashboard.putNumber("right", right);

		rightPosition = rightMotor2.getSelectedSensorPosition(0);
		SmartDashboard.putString("DB/String 2", "Right Position: " + rightMotor1.get());
		leftPosition = leftMotor2.getSelectedSensorPosition(0);
//		SmartDashboard.putString("DB/String 0", "Left Position: " + leftMotor1.get());
		System.out.println("left" + leftPosition);
		System.out.println("right" + rightPosition);

//		float yaw = navx.getYaw();
		yaw = cat.getAngle();
		SmartDashboard.putString("DB/String 4", "Z: " + (float)yaw);
//		SmartDashboard.putString("DB/String 5", "" + compressor.enabled());
		if (driverStick.isFirstLBPressed()) {
			gearShiftLeft.set(true);
			gearShiftRight.set(true);
		}
		else if (driverStick.isFirstRBPressed()) {
			gearShiftLeft.set(false);
			gearShiftRight.set(false);
		}

		if (driverStick.isFirstYPressed()) {
//			double currentYaw = navx.getYaw();

			cat.reset();
			
			float currentNavxYaw = navx.getYaw();
			cat.setAngleAdjustment(-currentNavxYaw);
			cat.nextAngle(currentNavxYaw);
			
			double currentYaw = cat.getAngle();
			resetNavxPID();
			navxPID.setSetpoint(currentYaw);
			SmartDashboard.putString("DB/String 5", "sp: " + (float)currentYaw);

			timer.reset();
		} else if (driverStick.isFirstXPressed()) {
/*
 * 			initialYaw = navx.getYaw();
			currentYaw = initialYaw;
			lastYaw = currentYaw;
			targetYaw = initialYaw + 75;
			if (targetYaw > 180) {
				targetYaw = (targetYaw - 360);
			}
			turnCompleted = false;
*/
			//turnLeft();
		}
		if (manipulatorStick.getRTValue() > .2 && !limitSwitchTop.get()) {
			armMotor1.set(manipulatorStick.getRTValue());
			armMotor2.set(-manipulatorStick.getRTValue());
		} else if (manipulatorStick.getLTValue() > .2 && !limitSwitchBottom.get()) {
			armMotor1.set(-manipulatorStick.getLTValue());
			armMotor2.set(manipulatorStick.getLTValue());
		} else {
			armMotor1.set(0);
			armMotor2.set(0);
		}
		SmartDashboard.putString("DB/String 7", "top limit switch: " + limitSwitchTop.get());
		SmartDashboard.putString("DB/String 8", "bottom limit switch: " + limitSwitchBottom.get());
		SmartDashboard.putString("DB/String 9", "Arm motor: " + armMotor2.get());

		if (Math.abs(manipulatorStick.getLYValue()) > .2) {
			leftIntake.set(-manipulatorStick.getLYValue());
		}
		else leftIntake.set(0);

		if (Math.abs(manipulatorStick.getRYValue()) > .2) {
			rightIntake.set(manipulatorStick.getRYValue());
		}
		else rightIntake.set(0);

		if (driverStick.isYHeldDown()) {
			SmartDashboard.putNumber("c", correction);
			double speedMultiplier = getMultiplier(timer.get(), 1.5);
			left = right = .8 * speedMultiplier;
			left = left + correction;  //// IF MOTORS HAVE BEEN FLIPPED, FLIP SIGNS
			right = right - correction;   // what that ^ says
			SmartDashboard.putString("DB/String 0", "L: " + left);
			SmartDashboard.putString("DB/String 1", "R: " + right);
			SmartDashboard.putString("DB/String 3", "corr " + (float)correction);
			
			m_drive.tankDrive(left, right);
		}
//		else if (driverStick.isBHeldDown()) {
//			double speedMultiplier = getSlowDownMultiplier(timer.get(), 1.5);
//			left = right = .8 * speedMultiplier;
//			left = left - correction;
//			right = right + correction;
//			m_drive.tankDrive(left, right);
//			SmartDashboard.putNumber("l", (leftSpeed - correction));
//			SmartDashboard.putNumber("r", (rightSpeed + correction));
//		} else if (driverStick.isXHeldDown()) {
//
////			if (currentYaw < targetYaw && !turnCompleted) {
////				m_drive.tankDrive(-.7, .7);
////				currentYaw = navx.getYaw();
////			} else if (initialYaw > 90 && currentYaw > 90 && !turnCompleted) {
////				m_drive.tankDrive(-.7, .7);
////				currentYaw = navx.getYaw();
////			} else if (lastYaw < targetYaw && currentYaw > targetYaw && !turnCompleted) {
////				turnCompleted = true;
////				m_drive.tankDrive(.3, -.3);
////				Timer.delay(.2);
////			}
//
////			else {
////				m_drive.tankDrive(0, 0);
////			}
//			lastYaw = currentYaw;
//		}
		else {
			left = -driverStick.getLYValue();
			right = -driverStick.getRYValue();
			m_drive.tankDrive(left, right);
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
		SmartDashboard.putString("DB/String 6", "PID Corr: " + output);
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

//		return navx.getYaw();
		return cat.getAngle();
	}

	public int inchesToTicks(double inches) {
		int ticks = (int) (inches * 4096 / 15.70794);
		return ticks;
	}

	public double getMultiplier(double time, double maxTime) {
		double multiplier = (Math.pow(time, 2)) / (Math.pow(maxTime, 2));
		if (multiplier < 0) {
			return 0;
		} else if (multiplier > 1) {
			return 1;
		} else {
			return multiplier;
		}
	}

	public double getSlowDownMultiplier(double time, double maxTime) {
		double multiplier = (Math.pow(time - maxTime, 2)) / (Math.pow(maxTime, 2));
		if (multiplier < 0 | time > maxTime) {
			return 0;
		} else if (multiplier > 1) {
			return 1;
		} else {
			return multiplier;
		}
	}

//////////////////////////////// DRIVE "STRAIGHT"
	public void driveDistance(int distance) {
		int ticks = inchesToTicks(distance);
		int count = leftMotor2.getSelectedSensorPosition(0);
		int target = count + ticks;
		double speedMultiplier = 0;
		timer.start();
		multiplier = 0;
		navxPID.setSetpoint(navx.getYaw());
		while (speedMultiplier < 1) {
			speedMultiplier = getMultiplier(timer.get(), 1);
			left = right = .8 * speedMultiplier;
			left = left - correction;
			right = right + correction;
			m_drive.tankDrive(left, right);
//			SmartDashboard.putString("DB/String 0", "acceleratin");
//			SmartDashboard.putString("DB/String 1", "" + speedMultiplier);
		}
		while ((target - leftMotor2.getSelectedSensorPosition(0)) > 10000) {
			m_drive.tankDrive(.8 - correction, .8 + correction);
//			SmartDashboard.putString("DB/String 1", "" + (target - leftMotor2.getSelectedSensorPosition(0)));
		}
		leftPosition = leftMotor2.getSelectedSensorPosition(0);
		rightPosition = rightMotor2.getSelectedSensorPosition(0);
		timer.reset();
		while (timer.get() < .1) {
			m_drive.tankDrive(-.4, -.4);
		}
		m_drive.tankDrive(0, 0);
		// while (speedMultiplier > 0) {
		// speedMultiplier = getSlowDownMultiplier(timer.get(), 2);
		// left = right = .8 * speedMultiplier;
		// left = left - correction;
		// right = right + correction;
		// m_drive.tankDrive(left, right);
		// SmartDashboard.putString("DB/String 0", "deceleratin");
		// SmartDashboard.putString("DB/String 1", "" + speedMultiplier);
		// }
//		SmartDashboard.putString("DB/String 0", "Auton finished");

		SmartDashboard.putString("DB/String 2", "" + (leftMotor2.getSelectedSensorPosition(0) - leftPosition));
	}
	
	public void resetNavxPID() {
		navxPID.reset();
		navxPID.setOutputRange(-0.5, .5);
		navxPID.setAbsoluteTolerance(.01);
		navxPID.setInputRange(-180, 180);
		navxPID.enable();
	}

////////////////////////////////////////// TURN LEFT  /////////////////////////////////////////////////
	
//	public void turnLeft() {
//		initialYaw = navx.getYaw();
//		SmartDashboard.putString("DB/String 5", "Turning finished: false");
//		SmartDashboard.putString("DB/String 7", "initial yaw: " + initialYaw);
//		currentYaw = initialYaw;
//		lastYaw = currentYaw;
//		targetYaw = initialYaw + 45;
//		if (targetYaw > 180) {
//			targetYaw -= 360;
//		}
//		turnCompleted = false;
//
//		while (!turnCompleted) {
//			currentYaw = navx.getYaw();
//			SmartDashboard.putString("DB/String 8", "current yaw: " + currentYaw);
//			SmartDashboard.putString("DB/String 5", "turning");
//			if (currentYaw < targetYaw) {
//				m_drive.tankDrive(-.7, .7);
//			} else if (initialYaw > 90 && currentYaw > 90) {
//				m_drive.tankDrive(-.7, .7);
//			} else if (lastYaw < targetYaw && currentYaw > targetYaw) {
//				//turnCompleted = true;
//				m_drive.tankDrive(.3, -.3);
//				Timer.delay(.2);
//			} else if (currentYaw > targetYaw - 10 && currentYaw < targetYaw + 10) {
//				turnCompleted = true;
//			}
//			lastYaw = currentYaw;
//		}
////		Timer.delay(.7);
////		while (currentYaw > (5 + targetYaw) && currentYaw < (targetYaw - 5)) {
////			m_drive.tankDrive(.5, -.5);
////			SmartDashboard.putString("DB/String 5", "correcting");
////		}
//		m_drive.tankDrive(0, 0);
//		SmartDashboard.putString("DB/String 5", "Turning finished: true");
//		lastYaw = currentYaw;
//	}
}