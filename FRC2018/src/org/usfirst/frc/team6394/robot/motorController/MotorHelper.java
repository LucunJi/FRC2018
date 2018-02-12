package org.usfirst.frc.team6394.robot.motorController;

public class MotorHelper {
	public static double applyDeadband(double speed, double deadband){
		if (speed < deadband && speed > -deadband)
			return 0;
		else
			return speed;
	}
	
	public static double applyMotionSmoother(double currentSpeed, double targetSpeed, double threshold) {
		if (currentSpeed - targetSpeed > threshold)
			return currentSpeed - threshold;
		else if (currentSpeed - targetSpeed < -threshold)
			return currentSpeed + threshold;
		else
			return targetSpeed;
	}
	
	public static double applyDirectionSmoother(double currentSpeed, double targetSpeed, double threshold) {
		if (currentSpeed * targetSpeed < 0 && 
				Math.abs(currentSpeed - targetSpeed) > threshold)
			return 0;
		else
			return targetSpeed;
	}
	
	public static double applyAllSmoother(double currentSpeed, double targetSpeed, double accelerationThreshold, double directionThreshold) {
		return applyMotionSmoother(currentSpeed, 
				applyDirectionSmoother(currentSpeed, targetSpeed, directionThreshold), accelerationThreshold);
	}
	
}
