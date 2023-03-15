package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.Logger;
import frc.ApriltagsCamera.PositionServer;

public abstract class PositionTracker implements Tracker{
    public PositionServer m_posServer;
    
	public static class PositionContainer{
		public double x, y;
		
		public PositionContainer(double x, double y) {
			this.x = x;
			this.y = y;
		}
	}

    public PositionTracker()
    {
		Logger.log("PositionTracker", 1, "PositionTracker()");
		
        m_posServer = new PositionServer();
		m_posServer.start();
    }

    abstract public Pose2d getPose2d(); 
    abstract public double getLeftEncoderPos();	
	abstract public double getRightEncoderPos();
    abstract public double getLeftEncoderVel();
	abstract public double getRightEncoderVel();
    abstract public void update(ApriltagsCamera frontCamera, ApriltagsCamera rearCamera);
}
