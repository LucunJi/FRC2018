package org.usfirst.frc.team6394.robot.base;

import java.net.ConnectException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

import static org.usfirst.frc.team6394.robot.Constants.*;
import static org.usfirst.frc.team6394.robot.base.MotorHelper.*;

public class SensorDifferentialBase {
	
	public enum DistanceSensor{
		AHRS, ENCODER
	}
	
	private final TalonSRX leftMotor;
	private final TalonSRX rightMotor;
	private DistanceSensor distanceSensor = DistanceSensor.AHRS;
	private final AHRS ahrs = new AHRS(Port.kMXP);
	
	private double deadband;
	private ControlMode controlMode;
	private double velocityCoefficient = 4096 * 500.0 / 600;
	private double accelerationThreshold = 0;
	private double directionThreshold = 0;
	private double turnDegreePgain = 0;
	private double turnDegreeDgain = 0;
	private double goStraightPgain = 0;
	private double goStraightDgain = 0;
	

	public SensorDifferentialBase(TalonSRX leftMotor, TalonSRX rightMotor) {
		leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		leftMotor.setSensorPhase(true);
		leftMotor.configNominalOutputForward(0, kTimeoutMs);
		leftMotor.configNominalOutputReverse(0, kTimeoutMs);
		leftMotor.configPeakOutputForward(1, kTimeoutMs);
		leftMotor.configPeakOutputReverse(-1, kTimeoutMs);
		
		rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		rightMotor.setSensorPhase(true);
		rightMotor.configNominalOutputForward(0, kTimeoutMs);
		rightMotor.configNominalOutputReverse(0, kTimeoutMs);
		rightMotor.configPeakOutputForward(1, kTimeoutMs);
		rightMotor.configPeakOutputReverse(-1, kTimeoutMs);
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	public void setDeadband(double deadband) {
		this.deadband = deadband;
	}
	
	public void setControlMode(ControlMode controlMode) {
		this.controlMode = controlMode;
	}
	
	public void setVelocityCoefficient(double velocityCoefficient) {
		this.velocityCoefficient = velocityCoefficient;
	}

	public void setDistanceSensor(DistanceSensor sensor){
		distanceSensor = sensor;
	}
	
	public void setAccelerationThreshold(double accelerationThreshold) {
		this.accelerationThreshold = accelerationThreshold;
	}
	
	public void setDirectionThreshold(double directionThreshold) {
		this.directionThreshold = directionThreshold;
	}
	
	public void setTurnDegreeDgain(double turnDegreeDgain) {
		this.turnDegreeDgain = turnDegreeDgain;
	}
	
	public void setTurnDegreePgain(double turnDegreePgain) {
		this.turnDegreePgain = turnDegreePgain;
	}
	
	public void setGoStraightDgain(double goStraightDgain) {
		this.goStraightDgain = goStraightDgain;
	}
	
	public void setGoStraightPgain(double goStraightPgain) {
		this.goStraightPgain = goStraightPgain;
	}
	
	public AHRS getAHRS(){
		return ahrs;
	}
	
	public void processSpeed(double leftSpeed, double rightSpeed) {
		leftSpeed = applyDeadband(leftSpeed, deadband);
		rightSpeed = applyDeadband(rightSpeed, deadband);
		if (controlMode == ControlMode.Velocity) {
			leftSpeed *= velocityCoefficient;
			rightSpeed *= velocityCoefficient;
		}
	
		leftSpeed = applyAllSmoother(leftMotor.getSelectedSensorVelocity(kPIDLoopIdx), leftSpeed, accelerationThreshold, directionThreshold);
		rightSpeed = applyAllSmoother(rightMotor.getSelectedSensorVelocity(kPIDLoopIdx), rightSpeed, accelerationThreshold, directionThreshold);
	
		leftMotor.set(controlMode, leftSpeed);
		rightMotor.set(controlMode, rightSpeed);
	}
	
	public void tankDrive(double leftSpeed, double rightSpeed){
		if (leftSpeed == rightSpeed) {
			goStraight(leftSpeed);
		} else {
			ahrs.reset();
			processSpeed(leftSpeed, rightSpeed);
		}
	}
	
	public void stop(){
		tankDrive(0, 0);
	}
	
	public void goStraightMeter(double speed, double targetDistance){
		switch (distanceSensor){
		case AHRS:
			ahrs.reset();
			break;
		case ENCODER:
			leftMotor.getIntegralAccumulator(kPIDLoopIdx);
			break;
		}
		double currentDistance = 0;
		while (currentDistance != targetDistance){
			switch (distanceSensor){
			case AHRS:
				currentDistance = ahrs.getDisplacementX();
				break;
			case ENCODER:
				break;
			}
		}
		//Unfinished
	}
	
	public void turnDegree(double speed, double degree) throws ConnectException {
		if (!ahrs.isConnected()) throw new ConnectException("Lose connection with AHRS!");
		
		double tempDeadband = deadband;
		setDeadband(0);
		int sign;
		
		ahrs.reset();
		if(degree>0)
			sign = 1;
		else if(degree<0)
			sign = -1;
		else
			return;
		
		while (ahrs.getAngle()<degree) {
//			if (turnDegreeDgain != 0 && turnDegreePgain != 0) 
				double _speed = speed * (degree - ahrs.getAngle()) * turnDegreePgain - ahrs.getRate() * turnDegreeDgain;
				_speed += 0.1;
			_speed *= sign;
			System.out.println(_speed);
			processSpeed(_speed,-_speed);
		}
		ahrs.reset();
		stop();
		setDeadband(tempDeadband);
	}
	
	public void goStraight(double speed) {
		double turnThrottle = 0;
		if (ahrs.isConnected()) {
			turnThrottle = -ahrs.getAngle() * goStraightPgain - ahrs.getRate() * goStraightDgain;
		}
		processSpeed(speed + turnThrottle, speed - turnThrottle);
	}
}
