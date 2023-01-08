/*
 *	  Copyright (C) 2020  John H. Gaby
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, version 3 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *    
 *    Contact: robotics@gabysoft.com
 */

package frc.ApriltagsCamera;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;


/**
 * 
 * @author John Gaby
 * 
 * @brief The ApriltagsCamera class handles communication with the Raspberry Pi Camera.
 * 
 *        This class allows you to connect to the Data server of the Raspberry
 *        Pi and receive information about the image frames processed by the
 *        camera.
 *
 *        It also implements a time sync protocol to synchronize the clocks and
 *        performs keep-alive functions that detect a disconnect.
 *
 */
public class ApriltagsCamera implements frc.ApriltagsCamera.Network.NetworkReceiver {
	/**
	 * @brief The ApriltagsCameraStats class collects the current camera performance
	 *        statistics.
	 *
	 */
	public class ApriltagsCameraStats {
		public int m_averageDelay = 0; //!<Specifies the average frame delay
		public int m_maxDelay = 0; //!<Specifies the max frame delay
		public int m_minDelay = Integer.MAX_VALUE; //!<Specifies the min frame delay
		public int m_lostFrames; //!<Specifies the number of lost frames
		public long m_time; //!<Specifies the elapsed time

		public ApriltagsCameraStats(int avgDelay, int maxDelay, int minDelay, int lostFrames, long time) {
			m_averageDelay = avgDelay;
			m_maxDelay = maxDelay;
			m_minDelay = minDelay;
			m_lostFrames = lostFrames;
			m_time = time;
		}
	}

	/**
	 * @brief The ApriltagsCameraPoint specifies an (x,y) position
	 *
	 */
	public class ApriltagsCameraPoint {
		public int m_x; //!<Specifies the horizontal position in pixels from the left
		public int m_y; //!<Specifies the vertical position in pixels from the top

		public ApriltagsCameraPoint(int x, int y) {
			m_x = x;
			m_y = y;
		}
	}

	/**
	 * @brief The AprilTagsCameraRect specifies an rectangular region
	 *
	 */
	public class AprilTagsCameraRect {
		public int m_left; //!<Specifies the left edge of the region in pixels
		public int m_top; //!<Specifies the top edge of the region in pixels
		public int m_right; //!<Specifies the right edge of the region in pixels
		public int m_bottom; //!<Specifies the bottom edge of the region in pixels

		public AprilTagsCameraRect(int left, int top, int right, int bottom) {
			m_left = left;
			m_top = top;
			m_right = right;
			m_bottom = bottom;
		}
	}

	/**
	 * @brief The ApriltagsCameraRegion specifies a single detected region
	 *
	 */
	public class ApriltagsCameraRegion {
		public int m_tag; //!<Specifies the region color [0..3]
		public double m_rvec[] = new double[3];
		public double m_tvec[] = new double[3];
		public double m_corners[][] = new double[4][2];

		// ! @cond PRIVATE
		public ApriltagsCameraRegion(int tag, double r0, double r1, double r2, double t0, double t1, double t2, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
			m_tag = tag;
			m_rvec[0] = r0;
			m_rvec[1] = r1;
			m_rvec[2] = r2;
			m_tvec[0] = t0;
			m_tvec[1] = t1;
			m_tvec[2] = t2;
			m_corners[0][0] = x0;
			m_corners[0][1] = y0;
			m_corners[1][0] = x1;
			m_corners[1][1] = y1;
			m_corners[2][0] = x2;
			m_corners[2][1] = y2;
			m_corners[3][0] = x3;
			m_corners[3][1] = y3;
		}
		// ! @endcond
	}

	/**
	 * @brief The ApriltagsCameraRegions specifies list of detected regions
	 *
	 *        The list will contain up to the max regions specified using the
	 *        ImageViewer and will be sorted from the largest to smallest area.
	 *
	 */
	public class ApriltagsCameraRegions {
		public int m_targetVertPos; //!<Specifies the vertical target position as set by the ImageViewer
		public int m_targetHorzPos; //!<Specifies the horizontal target position as set by the ImageViewer
		public int m_frameNo; //!<Specifies the frame #
		public int m_width; //!<Specifies the width of the camera image (e.g. 640)
		public int m_height; //!<Specifies the height of the camera image (e.g. 480)
		public int m_lostFrames; //!<Specifies the number of lost frames
		public int m_profile; //!<Specifies which capture profile is in effect
		public long m_captureTime; //!<Specifies the time at which the image was acquired in ms
		public int m_procTime; //!<Specifies the time required to process this image in ms
		public ArrayList<ApriltagsCameraRegion> m_regions = new ArrayList<ApriltagsCameraRegion>();

		// ! @cond PRIVATE
		protected ApriltagsCameraRegions(int frameNo, int targetVertPos, int targetHorzPos, int width, int height, int lostFrames,
				long captureTime, int procTime, int profile) {
			Logger.log("ApriltagsCameraRegions", -1, String.format("ApriltagsCameraRegions(): width = %d, height = %d", width, height));

			m_frameNo = frameNo;
			m_targetVertPos = targetVertPos;
			m_targetHorzPos = targetHorzPos;
			m_width = width;
			m_height = height;
			m_lostFrames = lostFrames;
			m_profile = profile;
			m_captureTime = captureTime;
			m_procTime = procTime;
		}

		protected void addRegion(int tag, double r0, double r1, double r2, double t0, double t1, double t2, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
			m_regions.add(new ApriltagsCameraRegion(tag, r0, r1, r2, t0, t1, t2, x0, y0, x1, y1, x2, y2, x3, y3));
		}
		// ! @endcond

		/**
		 * Returns a count of the regions
		 * 
		 */
		public int getRegionCount() {
			return (m_regions.size());
		}

		/**
		 * Returns a specified region
		 * 
		 * @param region - Specifies the region to return
		 */
		public ApriltagsCameraRegion getRegion(int region) {

			if ((region >= 0) && (region < m_regions.size())) {

				return (m_regions.get(region));
			};
			return (null);
		}
	}

	private static final int k_syncRetry = 5000;
	private static final int k_syncFirst = 1000;

	private Network m_network = null;
	private ApriltagsCameraRegions m_regions = null;
	private ApriltagsCameraRegions m_nextRegions = null;
	private long m_syncTime;
	private int m_averageDelayCount = 0;
	private int m_averageDelayMax = 30;
	private int m_averageDelaySum = 0;
	private int m_averageDelay = 0;
	private int m_maxDelay = 0;
	private int m_minDelay = Integer.MAX_VALUE;
	private int m_lostFrames;
	private int m_lastLostFrame = 0;
	private long m_startTime = 0;
	private boolean m_connected = false;
	private Timer m_watchdogTimer = new Timer();
	private long m_lastMessage;
	private static final int k_timeout = 5000;

	public ApriltagsCamera() {
		m_watchdogTimer.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
				if (m_connected) {
					Logger.log("ApriltagsCamera", -1, "WatchDog");

					m_network.sendMessage("k");

					if (m_lastMessage + k_timeout < System.currentTimeMillis()) {
						Logger.log("ApriltagsCamera", 3, "Network timeout");
						m_network.closeConnection();
					}
				}
			}
		}, 1000, 1000);
	}

	/**
	 * Returns true if connected to the camera
	 * 
	 */
	public boolean isConnected() {
		return (m_connected);
	}

	// ! @cond PRIVATE
	public static int[] parseIntegers(String str, int count) {
		int[] args = new int[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Integer.parseInt(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	public static long[] parseLong(String str, int count) {
		long[] args = new long[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Long.parseLong(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}
	
	public static double[] parseDouble(String str, int count) {
		double[] args = new double[count];
		int i = 0;

		String[] tokens = str.trim().split(" ");

		for (String token : tokens) {
			try {
				args[i] = Double.parseDouble(token);

			} catch (NumberFormatException nfe) {
				break;
			}

			if (++i >= count) {
				break;
			}
		}

		if (i == count) {
			return (args);
		}

		return (null);
	}

	long m_pingTime = 0;

	public void Ping() {
		m_pingTime = getTimeMs();

		m_network.sendMessage("p");
	}
	// ! @endcond

	/**
	 * Instructs the Raspberry Pi to dump the last N frame to disk. This can be
	 * useful in debugging to obtain a record of what the camera saw.
	 * 
	 * @param count - Count of frames to dump (max 120)
	 * 
	 */
	public void dumpFrames(int count) {
		Logger.log("ApriltagCamera", 1, String.format("DumpFrames(%d)", count));
		m_network.sendMessage(String.format("d %d", count));
	}

	PrintWriter m_log = null;
	long m_logTime = 0;

	/**
	 * Starts logging the data received from the Raspberry Pi locally
	 * 
	 * @param path - Specifies the path for the log file.
	 */
	public void startLogging(String path) {
		synchronized (this) {
			if (m_log != null) {
				m_log.close();
				m_log = null;
			}

			try {
				m_log = new PrintWriter(path);
				m_logTime = System.currentTimeMillis();
				m_lastLostFrame = m_lostFrames;
				m_minDelay = Integer.MAX_VALUE;
				m_maxDelay = 0;

				m_log.println("time,frame,#Regions,lost,cap delay,proc delay,x0,y0,x1,y1,x2,y2,x3,y3");
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Ends logging and closes the log file
	 * 
	 */
	public void endLogging() {
		synchronized (this) {
			if (m_log != null) {
				m_log.close();
				m_log = null;
			}
		}
	}

	/**
	 * if the Raspberry Pi is configured to controll the camera lights using
	 * pins 11 and 12, this command will turn the lights on or off
	 * 
	 * @param value - Bit zero controls pin 11 and bit one controls pin 12
	 */
	public void setLight(int value) {
		m_network.sendMessage(String.format("L %d", value));
	}
	
	// public void SendJpeg(boolean send)
	// {
	// 	m_network.SendMessage("J" + (send ? "y" : "n"));
	// }

	/**
	 * Specifies which capture profile should be used.
	 * 
	 * @param profile - Specifies the profile to use (0-3)
	 */
	public void setProfile(int profile) {
		if ((profile >= 0) && (profile < 4)) {
			m_network.sendMessage(String.format("P %d", profile));
		}
	}

	/**
	 * Instructs the Raspberry Pi to start logging the camera data on the Pi's disk
	 * This can be useful for debugging
	 *
	 */
	public void startPiLog() {
		m_network.sendMessage("ls");
	}

	/**
	 * Ends the Raspberry Pi logging
	 *
	 */
	public void endPiLog() {
		m_network.sendMessage("le");
	}

	/**
	 * Connects to the Raspberry Pi Data server
	 *
	 * @param host - Specifies IP for the host
	 * @param port - Specifies the port (default is 5800)
	 */
	public void connect(String host, int port) {
		m_network = new Network();

		m_startTime = System.currentTimeMillis();

		m_network.connect(this, host, port);

	}

	private void timeSync() {
		long time = getTimeMs();
		if (time > m_syncTime) {
			Logger.log("ApriltagCameras", -1, "TimeSync()");

			m_network.sendMessage(String.format("T1 %d", getTimeMs()));

			m_syncTime = time + k_syncRetry;
		}

	}

	private void processCameraFrame(String args) {
		long a[] = parseLong(args, 9);

		if (a != null) {
			m_nextRegions = new ApriltagsCameraRegions((int) a[0], (int) a[1], (int) a[2], (int) a[3], (int) a[4], (int) a[5],
					a[6], (int) a[7], (int) a[8]);

			int delay = (int) (System.currentTimeMillis() - m_nextRegions.m_captureTime);
			int averageDelay = -1;

			m_averageDelaySum += delay;
			if (++m_averageDelayCount >= m_averageDelayMax) {
				averageDelay = m_averageDelaySum / m_averageDelayCount;

				m_averageDelayCount = 0;
				m_averageDelaySum = 0;
			}

			synchronized (this) {
				if (averageDelay > 0) {
					m_averageDelay = averageDelay;
				}

				if (delay > m_maxDelay) {
					m_maxDelay = delay;
				}
				if (delay < m_minDelay) {
					m_minDelay = delay;
				}

				m_lostFrames = (int) a[5];
			}
		}
	}

	/**
	 * Returns the current camera performance stats
	 *
	 */
	public ApriltagsCameraStats getStats() {
		synchronized (this) {
			return (new ApriltagsCameraStats(m_averageDelay, m_maxDelay, m_minDelay, m_lostFrames - m_lastLostFrame,
					System.currentTimeMillis() - m_startTime));
		}
	}

	/**
	 * Clears the camera performance stats
	 *
	 */
	public void clearStats() {
		synchronized (this) {
			m_maxDelay = 0;
			m_minDelay = Integer.MAX_VALUE;
			m_lostFrames = 0;
			m_startTime = System.currentTimeMillis();
		}
	}

	private void processCameraRegion(String args) {
		double a[] = parseDouble(args, 15);

		if ((a != null) && (m_nextRegions != null)) {
			m_nextRegions.addRegion((int) a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14]);
		}
	}

	private void logFrame() {
		long curTime = System.currentTimeMillis();
		int size = m_regions.m_regions.size();

		m_log.print(String.format("%d,%d,%d,%d,%d,%d", curTime - m_logTime, m_regions.m_frameNo, size,
				m_regions.m_lostFrames, curTime - m_regions.m_captureTime, m_regions.m_procTime));

		if (size > 0) {
			ApriltagsCameraRegion region = m_regions.m_regions.get(0);

			for (int i = 0 ; i < 4 ; i++)
			{
				m_log.print(String.format(",%f,%f", region.m_corners[i][0],	region.m_corners[i][1]));
			}
		}

		m_log.println("");

		if ((m_regions.m_frameNo % 30) == 0) {
			m_log.flush(); // flush every 30 frames
		}
	}

	private void processCameraEnd(String args) {
		synchronized (this) {
			m_regions = m_nextRegions;
			m_nextRegions = null;

			if (m_log != null) {
				logFrame();
			}
		}

		timeSync();
	}

	/**
	 * Returns the latest set of camera regions. Note that the data is received from
	 * the camera in a separate thread so calling this twice in a row can generate a
	 * different result. Once you retrieve and instance of ApriltagsCameraRegions however,
	 * you can be assured that it will NOT be modified when a new frame is received.
	 *
	 */
	public ApriltagsCameraRegions getRegions() {
		synchronized (this) {
			return (m_regions);
		}

	}

	private long getTimeMs() {
		return System.currentTimeMillis();
	}

	private void processTimeSync() {
		Logger.log("ApriltagCamera", -1, "ProcessTimeSync()");

		long time = getTimeMs();

		m_network.sendMessage(String.format("T2 %d", time));
	}

	// ! @cond PRIVATE
	@Override
	public void processData(String data) {
		Logger.log("ApriltagCamera", -1, String.format("Data: %s", data));

		m_lastMessage = System.currentTimeMillis();

		switch (data.charAt(0)) {
		case 'F':
			processCameraFrame(data.substring(1).trim());
			break;

		case 'R':
			processCameraRegion(data.substring(1).trim());
			break;

		case 'E':
			processCameraEnd(data.substring(1).trim());
			break;

		case 'p':
			Logger.log("ApriltagsCamera", 3, String.format("Ping = %d", getTimeMs() - m_pingTime));
			break;

		case 'T': // sync
			processTimeSync();
			break;

		default:
			Logger.log("ApriltagsCamera", 3, String.format("Invalid command: %s", data));
			break;
		}
	}

	@Override
	public void disconnected() {
		m_connected = false;
	}

	@Override
	public void connected() {
		m_connected = true;

		m_syncTime = getTimeMs() + k_syncFirst;
		m_lastMessage = m_syncTime;
	}
	// ! @endcond
}
