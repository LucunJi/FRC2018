package org.usfirst.frc.team6394.robot.motorController;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;

public abstract class MotorRunnable implements Runnable {
	
	protected TalonGroup group;
	protected Spark spark;
	protected TalonSRX srx;
	protected VictorSPX spx;
	protected VictorSP sp;
	
	public MotorRunnable(TalonGroup group) {
		this.group = group;
	}
	
	public MotorRunnable(Spark spark) {
		this.spark = spark;
	}

	public MotorRunnable(TalonSRX srx) {
		this.srx = srx;
	}
	
	public MotorRunnable(VictorSPX spx) {
		this.spx = spx;
	}
	
	public MotorRunnable(VictorSP sp) {
		this.sp = sp;
	}
}
