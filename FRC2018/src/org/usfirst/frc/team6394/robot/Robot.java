/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6394.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.*;

import static org.usfirst.frc.team6394.robot.Constants.*;

import org.usfirst.frc.team6394.robot.base.SensorDifferentialBase;
import org.usfirst.frc.team6394.robot.motorController.MotorRunnable;
import org.usfirst.frc.team6394.robot.motorController.TalonGroup;

public class Robot extends IterativeRobot {
	XboxController xboxMotion = new XboxController(0);
	XboxController xboxFunction = new XboxController(1);
	
	TalonSRX leftTalon = new TalonSRX(0);
	TalonSRX rightTalon = new TalonSRX(2);
	VictorSPX leftVictor = new VictorSPX(1);
	VictorSPX rightVictor = new VictorSPX(3);
	SensorDifferentialBase base = new SensorDifferentialBase(leftTalon, rightTalon);
	
	TalonGroup intaker = new TalonGroup(new int[]{0,1});
	TalonGroup  lift = new TalonGroup(new int[]{2,3});
	TalonGroup intakerLift = new TalonGroup(new int[]{4,5});
	
	@Override	
	public void robotInit() {
		leftVictor.follow(leftTalon);
		rightVictor.follow(rightTalon);
		rightTalon.setInverted(true);
		rightVictor.setInverted(true);
		leftTalon.setNeutralMode(NeutralMode.Coast);
		rightTalon.setNeutralMode(NeutralMode.Coast);
		leftVictor.setNeutralMode(NeutralMode.Coast);
		rightVictor.setNeutralMode(NeutralMode.Coast);
		
		leftTalon.config_kF(kPIDLoopIdx, 0.4158, kTimeoutMs);
		leftTalon.config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		leftTalon.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		leftTalon.config_kD(kPIDLoopIdx, 4, kTimeoutMs);
		
		rightTalon.config_kF(kPIDLoopIdx, 0.4158, kTimeoutMs);
		rightTalon.config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		rightTalon.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		rightTalon.config_kD(kPIDLoopIdx, 4, kTimeoutMs);
		
		base.setControlMode(ControlMode.Velocity);
		base.setDeadband(0.1);
		base.setDirectionThreshold(900);
		base.setAccelerationThreshold(1100);
		base.setGoStraightPgain(0.01);
		base.setGoStraightDgain(0);
		base.setTurnDegreePgain(0.02);
		base.setTurnDegreeDgain(0);
	}
	
	private StringBuilder console = new StringBuilder();
	private int loops = 0;
	
	public void teleopInit() {
		base.getAHRS().reset();
	}
	
	@Override
	public void teleopPeriodic() {
		//follows the part of functional actions
		if (xboxMotion.getRawButton(3)) new Thread(new MotorRunnable(intaker) {
			@Override
			public void run() {
				group.set(-.3);
				Timer.delay(2);
				group.set(0);
			}
		}).start();
		if (xboxMotion.getRawButton(2)) intaker.set(.3);
		if (xboxMotion.getRawButton(4)) intaker.set(0);
		
		if(xboxMotion.getRawButton(5)) lift.set(-xboxMotion.getRawAxis(5));
		
		double s_intakerlift = xboxFunction.getRawAxis(5)*0.5;
    	intakerLift.set(s_intakerlift);
		if(xboxMotion.getRawButton(6)) intakerLift.set(s_intakerlift);
    	
		
		//follows the movement actions
		double xboxMotion_z = xboxMotion.getRawAxis(0)/4;
		
		double leftSpeed = (xboxMotion.getRawAxis(3)-xboxMotion.getRawAxis(2))/2 + xboxMotion_z;
		double rightSpeed = (xboxMotion.getRawAxis(3)-xboxMotion.getRawAxis(2))/2 - xboxMotion_z;
		
		base.tankDrive(leftSpeed, rightSpeed);
		
		
		if (++loops >= 8) {
			loops = 0;
			System.out.println(console.toString());
		}
		console.setLength(0);
	}
	
	
}
