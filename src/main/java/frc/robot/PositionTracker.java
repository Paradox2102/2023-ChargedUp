package frc.robot;

// import frc.lib.CSVWriter;
// import frc.lib.CSVWriter.Field;

public class PositionTracker implements Tracker{
	private double m_x;
	private double m_y;
		
	private double m_prevLeft;
	private double m_prevRight;
	
	private double m_lastAngle;
	
	private boolean m_runPositionThread = true;
	
	private Object lock = new Object();
	
	private PositionThread m_thread = new PositionThread();
	
	private SensorData m_sensors;

	// private Field[] k_fieldsPos = {
	// 	new Field("Function", 's'),
	// 	new Field("cur left", 'f'),
	// 	new Field("prev left", 'f'),
	// 	new Field("cur right", 'f'),
	// 	new Field("prev right", 'f'),
	// 	new Field("m_x", 'f'),
	// 	new Field("m_y", 'f')
	// };

	// private CSVWriter posWriter = new CSVWriter("Position Data", k_fieldsPos);

	public PositionTracker(double x, double y, boolean cameraCorrection, SensorData sensor) {
		m_sensors = sensor;
				
		m_prevLeft = getLeftEncoderPos();
		m_prevRight = getRightEncoderPos();
		m_lastAngle = getAngle();
		
		m_thread.start();
	}
	
	int count = 0;

	private void updatePos() {
		double leftPos = getLeftEncoderPos();
		double rightPos = getRightEncoderPos();
		double curAngle = getAngle();
		// System.out.println("Left Prev " + m_prevLeft + "\nRight Prev " + m_prevRight);
		// System.out.println("Left Pos " + leftPos + "\nRight " + rightPos);
		synchronized(lock) {

			double arcDist = ((leftPos - m_prevLeft) + (rightPos - m_prevRight))/2.0;
			
			double meanAngle = (curAngle + m_lastAngle)/2.0;
			double halfAngle =(curAngle - m_lastAngle)/2.0;

			meanAngle = Math.toRadians(meanAngle);
			halfAngle = Math.toRadians(halfAngle);

			double dist = arcDist * (1.0- (halfAngle * halfAngle)/6.0);
			
			

			// if (Math.abs(dist) > 0.001)
			// {
			// 	System.out.println(String.format("distxxx=%f", dist));
			// }
		
			m_x += (dist * Math.cos(meanAngle));
			m_y += (dist * Math.sin(meanAngle));

			// if ((count++ % 100) == 0)
			// {
			// 	System.out.println(String.format("PT: x=%f, y=%f, d=%f", m_x, m_y, dist));
			// 	System.out.println(String.format("PT: l=%f, y=%f", leftPos, rightPos));
			// }

			// posWriter.write("Update", ticksToFeet(leftPos), ticksToFeet(m_prevLeft), ticksToFeet(rightPos), ticksToFeet(m_prevRight), ticksToFeet(m_x), ticksToFeet(m_y));
			
	//		System.out.println(String.format("Dist: %f, x: %f, y%f", ticksToFeet(dist), ticksToFeet(m_x), ticksToFeet(m_y)));
			
			m_prevLeft = leftPos;
			m_prevRight = rightPos;
			m_lastAngle = curAngle;
			
	//		PositionWriter.write(ticksToFeet(leftPos), ticksToFeet(rightPos), ticksToFeet(dist), Math.toDegrees(curAngle), ticksToFeet(m_x), ticksToFeet(m_y));
		}
	}
	
	public PositionContainer getPos() {
		synchronized (lock) {
			return new PositionContainer(m_x, m_y);
		}
	}
	
	public void setXY(double x, double y) {
		synchronized (lock) {
			m_x = x;
			m_y = y;
			// posWriter.write("Set", ticksToFeet(getLeftEncoderPos()),ticksToFeet(m_prevLeft), ticksToFeet(getRightEncoderPos()), ticksToFeet(m_prevRight), ticksToFeet(m_x), ticksToFeet(m_y));
		}
	}
	
	public void setAngle(double angle) {
		synchronized (lock) {
			m_sensors.getGyro().setYaw(angle);
			m_lastAngle = angle;
		}
	}
	
	public void resetPreviousPosition() {
		synchronized (lock) {
			m_prevLeft = getLeftEncoderPos();
			m_prevRight = getRightEncoderPos();	
		}
	}
	
	public double getLeftEncoderPos() {
		return m_sensors.getLeftEncoderPos();
	}
	
	public double getRightEncoderPos() {
		return m_sensors.getRightEncoderPos();
	}

	public double getLeftEncoderVel() {
		return m_sensors.getLeftEncoderVel();
	}

	public double getRightEncoderVel() {
		return m_sensors.getRightEncoderVel();
	}
	
	public double getAngle() {
		return m_sensors.getAngle();
	}
	
	public static class PositionContainer{
		public double x, y;
		
		public PositionContainer(double x, double y) {
			this.x = x;
			this.y = y;
		}
	}
	
	public class PositionThread extends Thread{
		public void run() {
			long step = 20;
			for(long nextRun = System.currentTimeMillis();;nextRun += step) {
				if(m_runPositionThread) {
					updatePos();
				}
				try {
					long sleepTime = nextRun - System.currentTimeMillis();
					if(sleepTime > 0) {
						sleep(sleepTime);
					}
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
	
	public void startPosUpdate() {
		// posWriter.start();
		// cameraWriter.start();

		// posWriter.write("Start", ticksToFeet(getLeftEncoderPos()),ticksToFeet(m_prevLeft), ticksToFeet(getRightEncoderPos()), ticksToFeet(m_prevRight), ticksToFeet(m_x), ticksToFeet(m_y));
		m_runPositionThread = true;
	}
	
	public void stopPosUpdate() {
		// posWriter.finish();
		// cameraWriter.finish();

		m_runPositionThread = false;
	}
	
}