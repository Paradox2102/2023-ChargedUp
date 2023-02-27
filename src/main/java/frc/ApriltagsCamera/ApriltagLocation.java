package frc.ApriltagsCamera;

public class ApriltagLocation {
    int m_tag;
	public double m_xInches;
	public double m_yInches;
	public double m_targetAngleDegrees;
	// public int m_invalidCount = 0;

	public ApriltagLocation(int tag, double x, double y, double angle) {
		m_tag = tag;
		m_xInches = x;
		m_yInches = y;
		m_targetAngleDegrees = angle;
	}   
}
