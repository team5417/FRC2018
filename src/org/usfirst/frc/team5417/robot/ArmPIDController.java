package org.usfirst.frc.team5417.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class ArmPIDController extends PIDSubsystem {

	WPI_TalonSRX motor = new WPI_TalonSRX(0);
	public ArmPIDController(double p, double i, double d) {
		super(p, i, d);
		
	}

	@Override
	protected double returnPIDInput() {
		return motor.getSelectedSensorPosition(0);
	}

	@Override
	protected void usePIDOutput(double output) {
		motor.pidWrite(output);
		
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	

	

}
