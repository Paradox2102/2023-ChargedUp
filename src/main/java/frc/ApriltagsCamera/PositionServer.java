package frc.ApriltagsCamera;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.ApriltagsCamera.Network.NetworkReceiver;

public class PositionServer implements NetworkReceiver {
    private Network m_network = new Network();
    private Timer m_watchdogTimer = new Timer();
    private boolean m_connected = false;
    private double m_xPos = 0;
    private double m_yPos = 0;
    private boolean m_newPos = true;
    private Object m_lock = new Object();
    private Object m_recvLock = new Object();
    private Gyro m_gyro;

    double m_testX = 10*12;
    double m_testY = 0;
    double m_angle = 90;

    public void start(Gyro gyro) {
        m_gyro = gyro;
        m_network.listen(this, 5802);

        m_watchdogTimer.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
                // Logger.log("PositionServer", 1, "connected=" + m_connected);
				if (m_connected) {
					Logger.log("PositionServer",-1, "Send position");

                    {

                        // m_network.sendMessage(String.format("+%.2f %.2f %.2f\n", m_angle, m_testX, m_testY));

                        // m_testX += 1;
                        // m_testY += 0.5;
                        // m_angle += 1.0/5;
                    }

                    double xPos;
                    double yPos;
                    boolean newPos;

                    synchronized (m_lock)
                    {
                        xPos = m_xPos;
                        yPos = m_yPos;
                        newPos = m_newPos;

                        m_newPos = false;
                    }

                    if (newPos)
                    {
					    m_network.sendMessage(String.format("+%.2f %.2f %.2f\n", m_gyro.getAngle(), xPos, yPos));
                    }
                    else
                    {
                        m_network.sendMessage("-\n");       // keep alive
                    }

					// if (m_lastMessage + k_timeout < System.currentTimeMillis()) {
					// 	Logger.log("ApriltagsCamera", 3, "Network timeout");
					// 	m_network.closeConnection();
					// }
				}
			}
		}, 200, 200);   // Send current position 5 times a second
    }

    public void setPosition(double x, double y) {
        synchronized (m_lock) {
            m_xPos = x;
            m_yPos = y;
            m_newPos = true;
        }
    }

    public void setAllianceColor(boolean red)
    {
        m_network.sendMessage(String.format("c%c", red ? 'r' : 'b'));
    }

    public class BezierData {
        public double m_x1; // In feet
        public double m_y1; // In feet
        public double m_angle1; // In radians
        public double m_l1; // In feet
        public double m_x2; // In feet
        public double m_y2; // In feet
        public double m_angle2; // In radians
        public double m_l2; // In Feet

        public BezierData(double x1, double y1, double angle1, double l1, double x2, double y2, double angle2,
                double l2) {
            Logger.log("BezierData", 1, String.format("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", x1, y1, angle1, l1, x2, y2, angle2, l2));

            m_x1 = x1;
            m_y1 = y1;
            m_angle1 = angle1;
            m_l1 = l1;
            m_x2 = x2;
            m_y2 = y2;
            m_angle2 = angle2;
            m_l2 = l2;
        }
    }

    private BezierData[] m_data;
    private BezierData[] m_newData;
    private int m_nBezier;

    private void processPath(String path) {
        Logger.log("PositionServer", 1, String.format("processPath: %s", path));

        int[] args = ApriltagsCamera.parseIntegers(path, 1);

        if (args != null) {
            m_nBezier = 0;
            m_newData = new BezierData[args[0]];
        }

    }

    private void processBezier(String bezier) {
        Logger.log("PositionServer", 1, String.format("processBezier: cnt=%d %s", m_nBezier, bezier));

        if ((m_newData != null) && (m_nBezier < m_newData.length)) {
            double[] arg = ApriltagsCamera.parseDouble(bezier, 8);
            m_newData[m_nBezier] = new BezierData(arg[0], arg[1], arg[2], arg[3], arg[4], arg[5], arg[6], arg[7]);
        }
        else
        {
            m_newData = null;
            Logger.log("PositionServer", 3, String.format("Invalid Bezier Command: %s", bezier));
        }

        m_nBezier++;
    }

    private void processEnd(String end) {
        if (m_newData != null) {
            if (m_nBezier == m_newData.length) {
                synchronized (m_recvLock) {
                    m_data = m_newData;
                    m_newData = null;
                }
            }
            else {
                Logger.log("PositionServer", 3, String.format("Invalid Bezier Length: %d/%d", m_nBezier, m_newData.length));
            }
        }
    }

    @Override
    public void processData(String data) {
        Logger.log("PositionServer", -1, String.format("Data: %s", data));

        switch (data.charAt(0)) {
            case 'P': // path
                processPath(data.substring(1).trim());
                break;

            case 'B': // Bezier curve
                processBezier(data.substring(1).trim());
                break;

            case 'E': // End
                processEnd(data.substring(1).trim());
                break;

            case 'k':   // keep alive
                break;

            default:
                Logger.log("PositionServer", 3, String.format("Invalid command: %s", data));
                break;
        }
    }

    public BezierData[] getData()
    {
        BezierData[] data;

        synchronized(m_recvLock)
        {
            data = m_data;
            m_data = null;
        }

        return(data);
    }

    @Override
    public void connected() {
        Logger.log("PositionServer", 1, "connected");

        m_connected = true;
    }

    @Override
    public void disconnected() {
        Logger.log("PositionServer", 1, "disconnected");

        m_connected = false;
    }

}
