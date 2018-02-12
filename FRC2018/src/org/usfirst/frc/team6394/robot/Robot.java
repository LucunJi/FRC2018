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
import edu.wpi.first.wpilibj.GenericHID.Hand;

import static org.usfirst.frc.team6394.robot.Constants.*;

import java.net.ConnectException;

import org.usfirst.frc.team6394.robot.GameplayUtil.GamePlayHelper;
import org.usfirst.frc.team6394.robot.GameplayUtil.Position;
import org.usfirst.frc.team6394.robot.base.SensorDifferentialBase;
import org.usfirst.frc.team6394.robot.motorController.MotorRunnable;
import org.usfirst.frc.team6394.robot.motorController.TalonGroup;

public class Robot extends IterativeRobot {
	private XboxController xboxMotion = new XboxController(0);
	private XboxController xboxFunction = new XboxController(1);
	
	private TalonSRX leftTalon = new TalonSRX(0);
	private TalonSRX rightTalon = new TalonSRX(2);
	private VictorSPX leftVictor = new VictorSPX(1);
	private VictorSPX rightVictor = new VictorSPX(3);
	private SensorDifferentialBase base = new SensorDifferentialBase(leftTalon, rightTalon);
	
	private TalonGroup intaker = new TalonGroup(new int[]{0,1});
	private TalonGroup  lift = new TalonGroup(new int[]{2,3});
	private TalonGroup intakerLift = new TalonGroup(new int[]{4,5});
	private DigitalInput intakerLiftUpper = new DigitalInput(0);
	private DigitalInput intakerLiftLower = new DigitalInput(1);
	
	//Change this to adapt to different start position
	private final Position startPosition = Position.LEFT;
	
	
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
		base.setAngleApproacherFgain(0.003);
		base.setAngleApproacherPgain(0.02);
		base.setAngleApproacherIgain(0);
		base.setAngleApproacherDgain(0);
	}
	
	private StringBuilder console = new StringBuilder();
	private int loops = 0;
	
	@Override
	public void autonomousInit() {
		
	}
	
	@Override
	public void autonomousPeriodic() {
		Position platePosition = GamePlayHelper.getPlatePositionAt(Position.ALLIANCE);
		int sign = (platePosition == Position.RIGHT) ? 1 : 0;//positive for right side movement and vice versa
		if (platePosition == startPosition){
			
		} else if (platePosition != startPosition && startPosition != Position.MIDDLE) {
			
		} else {
			
		}
	}
	
	@Override
	public void teleopInit() {
		base.getAHRS().reset();
	}
	
	@Override
	public void teleopPeriodic() {
		//follows the part of functional actions
		
		//following is code for one-joystick operation
/*		if (xboxMotion.getRawButton(3)) intaker.set(-0.3);
		if (xboxMotion.getRawButton(2)) intaker.set(0.3);
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
		
		if(xboxMotion.getRawButton(5)) lift.set(-xboxMotion.getRawAxis(5)*0.5);
		
		double s_intakerlift = xboxMotion.getRawAxis(5)*0.5;
    	
		if(xboxMotion.getRawButton(6)) intakerLift.set(s_intakerlift);*/
		
		//following is code for two-joystick operation
		intaker.set(-xboxFunction.getRawAxis(1)*0.3);
		if (xboxFunction.getRawButton(1)) {
			lift.set(-(-xboxFunction.getTriggerAxis(Hand.kRight)+xboxFunction.getTriggerAxis(Hand.kLeft))*0.3);
		}else {
			lift.set(0);
		}
		if (xboxFunction.getRawButton(4)) {
			if(intakerLiftUpper.get()&xboxFunction.getTriggerAxis(Hand.kRight)>0) {
				intakerLift.set(0);
			}
			else if(intakerLiftLower.get()&xboxFunction.getTriggerAxis(Hand.kLeft)>0) {
				intakerLift.set(0);
			}
			else {
			intakerLift.set(-(xboxFunction.getTriggerAxis(Hand.kRight)-xboxFunction.getTriggerAxis(Hand.kLeft)*0.775)*0.3);
			}
		}else {
			intakerLift.set(0);
		}
    	
		
		//follows the movement actions
		if(xboxMotion.getRawButton(2)) base.turnAngle(90, kTurnTimeoutSec);
		
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
