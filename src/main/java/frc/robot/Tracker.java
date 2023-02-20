package frc.robot;

import frc.robot.PositionTracker.PositionContainer;

public interface Tracker {
	PositionContainer getPos();
	void setXYAngle(double x, double y, double angle);
}