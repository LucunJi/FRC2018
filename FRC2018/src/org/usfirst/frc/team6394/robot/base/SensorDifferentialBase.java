package org.usfirst.frc.team6394.robot.base;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;

import static org.usfirst.frc.team6394.robot.Constants.*;
import static org.usfirst.frc.team6394.robot.motorController.MotorHelper.*;

public class SensorDifferentialBase {
	
	private class ThrottleFetcher implements PIDOutput {
		private double throttle = 0;
		@Override public void pidWrite(double output) {
			throttle = output;
		}
		public double getThrottle() {
			return throttle;
		}
	}
	
	private final TalonSRX leftMotor;
	private final TalonSRX rightMotor;
	private final AHRS ahrs = new AHRS(Port.kMXP);
	
	private double deadband;
	private ControlMode controlMode;
	private double velocityCoefficient = 4096 * 500.0 / 600;
	private double accelerationThreshold = 0;
	private double directionThreshold = 0;
	
	private ThrottleFetcher throttleFetcher = new ThrottleFetcher();
	private PIDController straightKeeper;
	

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

		straightKeeper = new PIDController(0, 0, 0, 0, ahrs, throttleFetcher);
		straightKeeper.setInputRange(-180,180);
		straightKeeper.setOutputRange(-1,1);
		straightKeeper.setAbsoluteTolerance(0);
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
	
	public void setAccelerationThreshold(double accelerationThreshold) {
		this.accelerationThreshold = accelerationThreshold;
	}
	public void setDirectionThreshold(double directionThreshold) {
		this.directionThreshold = directionThreshold;
	}
	
	public PIDController getStraightKeeper() {
		return straightKeeper;
	}
	public void setStraightKeeperFgain(double f) {
		straightKeeper.setF(f);
	}
	public void setStraightKeeperPgain(double p) {
		straightKeeper.setP(p);
	}
	public void setStraightKeeperIgain(double i) {
		straightKeeper.setI(i);
	}
	public void setStraightKeeperDgain(double d) {
		straightKeeper.setD(d);
	}
	
	public AHRS getAHRS(){
		return ahrs;
	}
	
	public void processSpeed(double leftSpeed, double rightSpeed) {
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
		leftSpeed = applyDeadband(leftSpeed, deadband);
		rightSpeed = applyDeadband(rightSpeed, deadband);
		if (leftSpeed == rightSpeed && leftSpeed != 0 &&
				leftMotor.getSelectedSensorVelocity(kPIDLoopIdx) != 0 &&rightMotor.getSelectedSensorVelocity(kPIDLoopIdx) != 0) {
			if (!straightKeeper.isEnabled()) {
				straightKeeper.setSetpoint(ahrs.getYaw());
				straightKeeper.enable();
			}
			double throttle = throttleFetcher.getThrottle();
			if (throttle > 0.05) throttle = 0.05;
			if (throttle < -0.05) throttle = -0.05;
			leftSpeed += throttle;
			rightSpeed -= throttle;
		} else {
			if (straightKeeper.isEnabled()) straightKeeper.disable();
		}
		processSpeed(leftSpeed, rightSpeed);
	}
	
	public void stop(){
		tankDrive(0, 0);
	}
}
