package frc.robot;

import frc.robot.PositionTracker.PositionContainer;

public interface Tracker {
	PositionContainer getPos();
	void setXY(double x, double y);
	void setAngle(double angle);
	void resetPreviousPosition();
}