/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5417.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import org.usfirst.frc.team5417.robot.XBoxController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
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
public class Robot extends TimedRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(1);
	WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(2);
	WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(4);
	WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(5);
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor1, leftMotor2);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor1, rightMotor2);
	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
	XBoxController driverStick = new XBoxController(new Joystick(0));
	XBoxController manipulatorStick = new XBoxController(new Joystick(1));
	int leftPosition = 0;
	int rightPosition = 0;
	boolean buttonstatus = false;
	ArmPIDController Arm = new ArmPIDController(.2,0,0);
	double left;
	double right;
	AHRS navx = new AHRS(I2C.Port.kOnboard);
	float yaw = 0 ;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
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
		Arm.setOutputRange(-1,1);
		Arm.setAbsoluteTolerance(10);
		Arm.enable();
		m_autoSelected = kDefaultAuto ;
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
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				m_drive.tankDrive(.3, -.3);
			while(yaw < 90) {
				yaw = navx.getYaw();
				SmartDashboard.putString("DB/String 4", "Z axis: " + yaw);
			}
			m_drive.tankDrive(0, 0);
				break;
			case kDefaultAuto:
			default:
				navx.reset();
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
		//Arm.setSetpoint(motor.getSelectedSensorPosition(0));
	}
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		left = -driverStick.getLYValue();
		right = -driverStick.getRYValue();
		m_drive.tankDrive(left,right);
		SmartDashboard.putNumber("left", left);
		SmartDashboard.putNumber("right", right);
		
		rightPosition = rightMotor1.getSelectedSensorPosition(0);
		SmartDashboard.putString("DB/String 2", "Right Position: " + rightPosition);
		leftPosition = leftMotor1.getSelectedSensorPosition(0);
		SmartDashboard.putString("DB/String 0", "Left Position: " + leftPosition);
		
		float yaw = navx.getYaw();
		SmartDashboard.putString("DB/String 4", "Z axis: " + yaw);
	}
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
