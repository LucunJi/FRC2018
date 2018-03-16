package org.usfirst.frc.team6394.robot.GameplayUtil;

import edu.wpi.first.wpilibj.DriverStation;

public class GamePlayHelper {
	public static Position getPlatePositionAt(final Position pos) {
		int index;
		switch (pos) {
		case ALLIANCE:
			index = 0;
			break;
		case MIDDLE:
			index = 1;
			break;
		case OPPONENT:
			index = 2;
			break;
		default:
			throw new IllegalArgumentException("Argument can only be ALLIANCE, MIDDLE or OPPONENT!");
		}
		char c = DriverStation.getInstance().getGameSpecificMessage().charAt(index);
		DriverStation.getInstance().getLocation();
		if (c == 'L') {
			return Position.LEFT;
		} else {
			return Position.RIGHT;
		}
	}
}
