package org.usfirst.frc.team6394.robot.base;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;

import static org.usfirst.frc.team6394.robot.Constants.*;
import static org.usfirst.frc.team6394.robot.motorController.MotorHelper.*;

public class SensorDifferentialBase {
	
	public enum SensorType{
		AHRS, CTRE_MAGENCODER, ULTRASONICS
	}
	
	private class ThrottleFetcher implements PIDOutput {
		private double throttle = 0;
		@Override public void pidWrite(double output) {
			throttle = output;
		}
		public double getThrottle() {
			return throttle;
		}
	}

	private class displacementPIDOutput implements PIDOutput {
	    private SensorDifferentialBase base;
	    public displacementPIDOutput(SensorDifferentialBase base) { this.base = base; }
        @Override public void pidWrite(double output) {
	        PIDController angleApproacher = base.getAngleApproacher();
	        if (!angleApproacher.isEnabled()) {
	            angleApproacher.enable();
            }
            processSpeed(output,output);
        }
    }
	private final TalonSRX leftMotor;
	private final TalonSRX rightMotor;
	private SensorType distanceSensor = SensorType.AHRS;
	private final AHRS ahrs = new AHRS(Port.kMXP);
	
	private double deadband;
	private ControlMode controlMode;
	private double velocityCoefficient = 4096 * 500.0 / 600;
	private double accelerationThreshold = 0;
	private double directionThreshold = 0;
	
	private ThrottleFetcher throttleFetcher = new ThrottleFetcher();
	private PIDController angleApproacher;
	

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
		
		angleApproacher = new PIDController(0, 0, 0, 0, ahrs, throttleFetcher);
		angleApproacher.setAbsoluteTolerance(1);
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

	public void setDistanceSensor(SensorType sensor){
		distanceSensor = sensor;
	}
	
	public void setAccelerationThreshold(double accelerationThreshold) {
		this.accelerationThreshold = accelerationThreshold;
	}
	public void setDirectionThreshold(double directionThreshold) {
		this.directionThreshold = directionThreshold;
	}
	
	public PIDController getAngleApproacher() {
		return angleApproacher;
	}
	public void setAngleApproacherFgain(double f) {
		angleApproacher.setF(f);
	}
	public void setAngleApproacherPgain(double p) {
		angleApproacher.setP(p);
	}
	public void setAngleApproacherIgain(double i) {
		angleApproacher.setI(i);
	}
	public void setAngleApproacherDgain(double d) {
		angleApproacher.setD(d);
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
			if (!angleApproacher.isEnabled()) {
				angleApproacher.setSetpoint(ahrs.getYaw());
				angleApproacher.enable();
			}
			double throttle = throttleFetcher.getThrottle();
			leftSpeed += throttle;
			rightSpeed -= throttle;
		} else {
			if (angleApproacher.isEnabled()) angleApproacher.disable();
		}
		processSpeed(leftSpeed, rightSpeed);
	}
	
	public void stop(){
		tankDrive(0, 0);
	}
	
	public void turnAngle(double angle, double timeoutSeconds) {
		angleApproacher.setSetpoint(angleApproacher.getSetpoint() + angle);
		if (!angleApproacher.isEnabled()) angleApproacher.enable();
		Timer timer = new Timer();
		timer.start();
		while (timer.get() < timeoutSeconds && 
				!(leftMotor.getSelectedSensorVelocity(kPIDLoopIdx) == 0 && rightMotor.getSelectedSensorVelocity(kPIDLoopIdx) == 0)) {
			tankDrive(0, 0);
		}
		timer.stop();
	}
	
	public void displacementMove(double displacement, double timeoutSec){
		switch (distanceSensor){
			case AHRS:
				double targetDisplacement = ahrs.getDisplacementX() + displacement;
				AHRS displacementAHRS = new AHRS(Port.kMXP){
					@Override
					public double pidGet(){
						return this.getDisplacementX();
					}
				};
				PIDController displacementController = new PIDController(0,0,0,0.3,
                        displacementAHRS, new displacementPIDOutput(this));
				displacementController.setAbsoluteTolerance(0.03);
				Timer timer = new Timer();
				displacementController.enable();
				timer.start();
				while (!(leftMotor.getSelectedSensorVelocity(kPIDLoopIdx) == 0 && rightMotor.getSelectedSensorVelocity(kPIDLoopIdx) == 0)
                        && timer.get() < timeoutSec) {
                    Timer.delay(0.01);
				}
				break;
			default:
				throw new IllegalArgumentException("Unsupported sensor type for distance measurement!");
		}
	}
}
