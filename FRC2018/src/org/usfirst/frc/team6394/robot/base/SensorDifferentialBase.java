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
	
	private final TalonSRX leftMotor;
	private final TalonSRX rightMotor;
	private final AHRS ahrs = new AHRS(Port.kMXP);
	
	private double deadband;
	private ControlMode controlMode;
	private double velocityCoefficient = 4096 * 500.0 / 600;
	private double accelerationThreshold = 0;
	private double directionThreshold = 0;
	

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
	
	public void setAccelerationThreshold(double accelerationThreshold) {
		this.accelerationThreshold = accelerationThreshold;
	}
	public void setDirectionThreshold(double directionThreshold) {
		this.directionThreshold = directionThreshold;
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
		processSpeed(leftSpeed, rightSpeed);
	}
	
	public void stop(){
		tankDrive(0, 0);
	}
}
