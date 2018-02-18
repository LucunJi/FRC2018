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

import com.kauailabs.navx.frc.AHRS;
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
	private DigitalInput LiftUpper = new DigitalInput(2);
	private DigitalInput LiftLower = new DigitalInput(3);
	//Change this to adapt to different start position
	private final Position startPosition = Position.LEFT;
	double forwardLeftPercentage = 0.396;
	double forwardRightPercentage = 0.385;
	double backwardLeftPercentage = 0;
	double backwardRightPercentage = 0;
	double turningLeftPercentage = -0.2;
	double turningRightPercentage = 0.2;
	
	
	
	@Override	
	public void robotInit() {
		leftVictor.follow(leftTalon);
		rightVictor.follow(rightTalon);
		rightTalon.setInverted(true);
		rightVictor.setInverted(true);
		leftTalon.setNeutralMode(NeutralMode.Brake);
		rightTalon.setNeutralMode(NeutralMode.Brake);
		leftVictor.setNeutralMode(NeutralMode.Brake);
		rightVictor.setNeutralMode(NeutralMode.Brake);
		
		leftTalon.config_kF(kPIDLoopIdx, 0.4158, kTimeoutMs);
		leftTalon.config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		leftTalon.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		leftTalon.config_kD(kPIDLoopIdx, 4, kTimeoutMs);
		
		rightTalon.config_kF(kPIDLoopIdx, 0.4158, kTimeoutMs);
		rightTalon.config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		rightTalon.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		rightTalon.config_kD(kPIDLoopIdx, 4, kTimeoutMs);

		base.setDeadband(0.1);
		base.setDirectionThreshold(1100);
		base.setAccelerationThreshold(1100);
	}
	
	private StringBuilder console = new StringBuilder();
	private int loops = 0;
	
	@Override
	public void autonomousInit() {
		base.getAHRS().reset();
		base.setControlMode(ControlMode.Velocity);
	}

	boolean flag = false;
	@Override
	public void autonomousPeriodic() {
		if (flag) {
			base.stop();
			return;
		}
		Position platePosition = GamePlayHelper.getPlatePositionAt(Position.ALLIANCE);
		int sign = (platePosition == Position.LEFT) ? 1 : -1;//positive for left side movement and vice versa
		if (platePosition == startPosition){// left to left or right to right
			//driving to the preparation position
			
			
			base.tankDrive(forwardLeftPercentage, forwardRightPercentage);
			Timer.delay(4.3);
			base.stop();
			Timer.delay(1.5);
			
			
			//turn 90 degrees left
			base.tankDrive(turningLeftPercentage, turningRightPercentage);
			Timer.delay(0.72);
			base.tankDrive(0, 0);
//			Timer.delay(0.2);
			
			
			//driving 1.181m to the second turning point
			base.tankDrive(forwardLeftPercentage, forwardRightPercentage);
			Timer.delay(1.1655);
			base.tankDrive(0, 0);
//			Timer.delay(0.2);
			//turn 90 degrees left
			base.tankDrive(turningLeftPercentage, turningRightPercentage);
			Timer.delay(0.72);
			base.tankDrive(0, 0);
			
			
			//enable intaker
			intaker.set(-1);
			//drive 0.7m forward to get the cube
			base.tankDrive(forwardLeftPercentage, forwardRightPercentage);
			Timer.delay(0.9);
			base.tankDrive(0, 0);
			Timer.delay(0.1);
			
			intaker.set(0);
			
			Timer.delay(0.5);
			
			//lifting the intaker
			intakerLift.set(-1);
			Timer.delay(0.5);
			intakerLift.set(-0.1);
			//driving forward a little to touch the wall
			base.tankDrive(forwardLeftPercentage, forwardRightPercentage);
			Timer.delay(0.5);
			base.tankDrive(0, 0);
//			Timer.delay(0.2);
			//shooting the cube
			intaker.set(0.3);
			Timer.delay(0.5);
			intaker.set(0);
		} else if (platePosition != startPosition && startPosition != Position.MIDDLE) {
			//left to right or right to left
			System.out.println("B");
		} else {
			//middle to left or right
			System.out.println("C");
		}
		flag = true;
		base.setControlMode(ControlMode.Velocity);
	}

	private boolean pastButton1State;
	@Override
	public void teleopInit() {
		base.getAHRS().reset();
		intakerLift.set(-0.1);
		pastButton1State = xboxMotion.getRawButton(1);
		base.setControlMode(ControlMode.Velocity);
	}

	@Override
	public void teleopPeriodic() {
		if (isEnabled() && isOperatorControl()) {
			
			//follows the part of functional actions

			//following is code for one-joystick operation
/*			if(xboxMotion.getRawButton(3)){
				intaker.set(-1);
			}else if(xboxMotion.getRawButton(2)){
				intaker.set(1);
			}else if (xboxMotion.getRawButton(4)) {
				intaker.set(0);
				
			}
			if (xboxMotion.getRawButton(5)) {
				if (LiftUpper.get() && xboxMotion.getRawAxis(5) != 0) {
					lift.set(0);
				} else if (LiftLower.get() && xboxMotion.getRawAxis(5) != 0) {
					lift.set(0);
				}else {
					lift.set(-(-xboxMotion.getRawAxis(5)) * 0.3);
				}
			} else {
				lift.set(0);
			}
			if (xboxMotion.getRawButton(6)) {
				if (intakerLiftUpper.get() && xboxMotion.getRawAxis(5) < 0) {
					intakerLift.set(-0.1);
				} else if (intakerLiftLower.get() && xboxMotion.getRawAxis(5) > 0) {
					intakerLift.set(-0.1);
				}else if(xboxMotion.getRawAxis(5)>-0.1 && xboxMotion.getRawAxis(5)<0.1){
					
					intakerLift.set(-0.1);
				}else {
					intakerLift.set(xboxMotion.getRawAxis(5)*0.4);
				}
			} else {
				intakerLift.set(-0.1);
			}


			
			if (xboxMotion.getRawButton(1)) {
				//put something to test
			}*/

			//following is code for two-joystick operation
			intaker.set(-xboxFunction.getRawAxis(1) * 0.5);
			if (xboxFunction.getRawButton(1)) {
				if (LiftUpper.get() && xboxFunction.getTriggerAxis(Hand.kRight) >= 0) {
					lift.set(0);
				} else if (LiftLower.get() && xboxFunction.getTriggerAxis(Hand.kLeft) >= 0) {
					//suosi
				}else {
					lift.set(-(-xboxFunction.getTriggerAxis(Hand.kRight) + xboxFunction.getTriggerAxis(Hand.kLeft))*0.5 );
				}
			} else {
				lift.set(0);
			}
			if (xboxFunction.getRawButton(4)) {
				if ((intakerLiftUpper.get() && xboxFunction.getTriggerAxis(Hand.kRight) > 0) ||
						(intakerLiftLower.get() && xboxFunction.getTriggerAxis(Hand.kLeft) > 0)) {
					intakerLift.set(0);
				} else if (xboxFunction.getTriggerAxis(Hand.kRight) == 0 && xboxFunction.getTriggerAxis(Hand.kLeft) == 0) {
					intakerLift.set(-0.0675);
				} else {
					intakerLift.set(-(xboxFunction.getTriggerAxis(Hand.kRight) - xboxFunction.getTriggerAxis(Hand.kLeft))*0.5 );
				}
			} else {
				intakerLift.set(0);
			}


			
//			if (xboxMotion.getRawButton(2)) {
//				//put something to test
//				rotateAngle(90, 10);
//			}

			double xboxMotion_z = xboxMotion.getRawAxis(0) / 4;

			double leftSpeed = (xboxMotion.getRawAxis(3) - xboxMotion.getRawAxis(2))*0.65  + xboxMotion_z;
			double rightSpeed = (xboxMotion.getRawAxis(3) - xboxMotion.getRawAxis(2)) *0.65 - xboxMotion_z;

			if (xboxMotion.getRawButton(1) && !pastButton1State)
				base.getAHRS().reset();
			if (xboxMotion.getRawButton(1) && base.getAHRS().isConnected()) {
				double throttle = (0 - base.getAHRS().getAngle()) * 0.01;
				if (throttle > 0.05) throttle = 0.05;
				if (throttle < -0.05) throttle = -0.05;
				leftSpeed += throttle;
				rightSpeed -= throttle;
			}


			base.tankDrive(leftSpeed, rightSpeed);

			console.append(base.getAHRS().isConnected() +
					"\t" + base.getAHRS().getAngle()
			);


			if (++loops >= 8) {
				loops = 0;
				System.out.println(console.toString());
			}
			console.setLength(0);

			pastButton1State = xboxMotion.getRawButton(1);
		}
	}

	/*public void rotateAngle(double angle, double timeoutSec) {
		int sign = angle > 0 ? 1 : -1;
		AHRS ahrs = base.getAHRS();
		double trgAngle = ahrs.getAngle() + angle;
		Timer timer = new Timer();
		timer.start();
		while (sign * ahrs.getAngle() < sign * trgAngle && isEnabled() && timer.get() < timeoutSec) {
			double error = trgAngle - ahrs.getAngle();
			error *= sign;
			double throttle = (error)/angle/8+ 1*sign;
			base.processSpeed(throttle,-throttle);
		}
		timer.stop();
		base.stop();
	}

	public void moveDistance(double distance, double timeoutSec) {
		int sign = distance > 0 ? 1 : -1;
		int currentPosition = leftTalon.getSelectedSensorPosition(kPIDLoopIdx);
		double trgPosition = currentPosition + distance * 8200;
		Timer timer = new Timer();
		timer.start();
		while (sign * currentPosition < sign * trgPosition && isEnabled() && timer.get() < timeoutSec) {
			currentPosition = leftTalon.getSelectedSensorPosition(kPIDLoopIdx);
			double error = trgPosition - currentPosition;
			error *= sign;
			double throttle;
			if (Math.abs(error/8200) < 1) {
				throttle = error / (distance * 8400) / 8 + 0.1 * sign;
			} else {
				throttle = error / (distance * 8400) / 4 + 0.1 * sign;
			}
			base.tankDrive(throttle,throttle);
		}
		base.stop();
	}*/
}
