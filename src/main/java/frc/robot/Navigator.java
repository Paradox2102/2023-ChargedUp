package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.ApriltagsCamera.Logger;

public class Navigator {
    PositionTracker m_tracker;
    Object m_lock = new Object();

    public Navigator(PositionTracker pos) {
        m_tracker = pos;
    }

    public class NavigatorPos {
        public final double yaw;
        public final double x;
        public final double y;
        public final double leftPos;
        public final double rightPos;
        public final float leftVel;
        public final float rightVel;

        private NavigatorPos(double yaw, double x, double y, double leftPos, double rightPos, float leftVel, float rightVel) {
            this.yaw = yaw;
            this.x = x;
            this.y = y;
            this.leftPos = leftPos;
            this.rightPos = rightPos;
            this.leftVel = leftVel;
            this.rightVel = rightVel;
        }
    }

    public void reset(double angle, double x, double y) {
        Logger.log("Navigator", 1, String.format("Reset:a=%f,x=%f,y=%f", angle, x, y));
        m_tracker.setXYAngle(x, y, angle);
    }

    public NavigatorPos getPos() {
        NavigatorPos pos;
        Pose2d con = m_tracker.getPose2d();
        synchronized(m_lock) {
            pos = new NavigatorPos(con.getRotation().getDegrees(), con.getX(), con.getY(), m_tracker.getLeftEncoderPos(), m_tracker.getRightEncoderPos(), (float) m_tracker.getLeftEncoderVel(), (float)m_tracker.getRightEncoderVel());
        }
        return pos;
    }

    // public Pose2d getPose2d() {
    //     NavigatorPos pos = getPos();
    //     return new Pose2d(pos.x * 0.0254 * 12, pos.x * 0.0254 * 12, Rotation2d.fromDegrees(pos.yaw));
    // }
}