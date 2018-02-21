package org.usfirst.frc.team5417.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class ArmPID extends PIDSubsystem {
	WPI_TalonSRX armMotor1 = new WPI_TalonSRX(3);
	double correction = 0;
	public ArmPID(double p, double i, double d) {
		super(p, i, d);
		
	}

	@Override
	protected double returnPIDInput() {
		return armMotor1.getSelectedSensorPosition(0);
	}

	@Override
	protected void usePIDOutput(double output) {
		correction = output;
		
	}
	double getOutput() {
		return correction;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	

	

}
