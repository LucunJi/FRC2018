/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6394.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.I2C.Port;

import com.kauailabs.navx.frc.AHRS;

import static org.usfirst.frc.team6394.robot.Constants.*;

public class Robot extends IterativeRobot {
	
	XboxController xboxMotion = new XboxController(0);
	XboxController xboxAction = new XboxController(1);
	
	TalonSRX t_l = new TalonSRX(0);
	TalonSRX t_r = new TalonSRX(2);
	VictorSPX v_l = new VictorSPX(1);
	VictorSPX v_r = new VictorSPX(3);
	
	Talon intaker_l = new Talon(0);
	Talon intaker_r = new Talon(1);
	Talon lift_l = new Talon(2);
	Talon lift_r = new Talon(3);
	Talon intakerLift_l = new Talon(4);
	Talon intakerLift_r = new Talon(5);
	
	AHRS ahrs = new AHRS(Port.kMXP);
	@Override
	
	public void robotInit() {
		v_l.follow(t_l);
		v_r.follow(t_r);
		t_r.setInverted(true);
		v_l.setInverted(true);
		
		t_l.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		t_l.setSensorPhase(true);
		t_l.configNominalOutputForward(0, kTimeoutMs);
		t_l.configNominalOutputReverse(0, kTimeoutMs);
		t_l.configPeakOutputForward(1, kTimeoutMs);
		t_l.configPeakOutputReverse(-1, kTimeoutMs);
		
		t_r.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		t_l.setSensorPhase(true);
		t_r.configNominalOutputForward(0, kTimeoutMs);
		t_r.configNominalOutputReverse(0, kTimeoutMs);
		t_r.configPeakOutputForward(1, kTimeoutMs);
		t_r.configPeakOutputReverse(-1, kTimeoutMs);
		
		t_l.config_kF(kPIDLoopIdx, 0, kTimeoutMs);
		t_l.config_kP(kPIDLoopIdx, 0, kTimeoutMs);
		t_l.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		t_l.config_kD(kPIDLoopIdx, 0, kTimeoutMs);
		
		t_r.config_kF(kPIDLoopIdx, 0, kTimeoutMs);
		t_r.config_kP(kPIDLoopIdx, 0, kTimeoutMs);
		t_r.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		t_r.config_kD(kPIDLoopIdx, 0, kTimeoutMs);
	}
	
	private StringBuilder console = new StringBuilder();
	private int loops = 0;
	
	@Override
	public void teleopPeriodic() {
		double xboxMotion_y = -xboxMotion.getRawAxis(1)/1.5;
		//xboxMotion_y = (xboxMotion_y < 0.08 ? (xboxMotion_y < -0.08 ? xboxMotion_y : 0) : xboxMotion_y);
		double xboxMotion_z = xboxMotion.getRawAxis(4)/2;
		
		double l_trg = xboxMotion_y + xboxMotion_z;
		double r_trg = xboxMotion_y - xboxMotion_z;
		
		console.append(t_l.getMotorOutputPercent());
		console.append("\t" + t_l.getSelectedSensorVelocity(kPIDLoopIdx));
		console.append("\t" + t_r.getMotorOutputPercent());
		console.append("\t" + t_r.getSelectedSensorVelocity(kPIDLoopIdx));
		
		t_l.set(ControlMode.PercentOutput, l_trg);
		t_r.set(ControlMode.PercentOutput, r_trg);
		
		if (++loops >= 8) {
			loops = 0;
			System.out.println(console.toString());
		}
		console.setLength(0);
	}
	
	
	private void turnDegree(double degree) {
		int sign;
		ahrs.reset();
		if(degree>0) {
			sign = 1;
		}else if(degree<0){
			sign = -1;
		}else {
			return;
		}
		while (sign*ahrs.getAngle()<sign*degree) {
			double spd = sign * ((degree - ahrs.getAngle())/degree*0.3 + 0.3);
			t_l.set(ControlMode.PercentOutput, spd);
			t_r.set(ControlMode.PercentOutput, -spd);
		}
		t_l.set(ControlMode.PercentOutput, 0);
		t_r.set(ControlMode.PercentOutput, 0);
	}
}
