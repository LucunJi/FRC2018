package org.usfirst.frc.team6394.robot.motorController;

import edu.wpi.first.wpilibj.Talon;

public class TalonGroup {
	
	private final Talon[] group;
	
	public TalonGroup (int[] ports) {
		group = new Talon[ports.length];
		for (int i = 0; i < ports.length; i++) {
			group[i] = new Talon(ports[i]);
			group[i].setInverted(false);
		}
	}
	
	public TalonGroup (int[] ports, boolean[] inverted) {
		group = new Talon[ports.length];
		for (int i = 0; i < ports.length; i++) {
			group[i] = new Talon(ports[i]);
			group[i].setInverted(inverted[i]);
		}
	}
	
	public void set(double speed){
		for (Talon t : group) {
			t.set(speed);
		}
	}
}
