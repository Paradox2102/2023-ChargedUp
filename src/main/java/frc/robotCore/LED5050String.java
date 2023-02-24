package frc.robotCore;

// import static org.junit.Assert.assertNotNull;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * 
 * @author John Gaby
 * 
 * @brief This class controls a string of individual LED lights
 * 
 *
 */
public class LED5050String 
{
	final private static int k_updateTime = 20;
	final private static ScheduledExecutorService m_scheduler = Executors.newScheduledThreadPool(1);
	final private static Object m_lock = new Object();

	static private class ArduinoConnection
	{
		private SerialPort m_port = new SerialPort(115200, SerialPort.Port.kMXP);

		public void SendCommand(String msg)
		{
			m_port.writeString(msg + "\n");
			// System.out.println(String.format("count = %d", count));
		}
	}
	
	/**
	 * 
	 * @author John Gaby
	 * 
	 * @brief Specifies a color as RGB
	 * 
	 */
	static public class RGBColor
	{
		public int	m_red;
		public int  m_green;
		public int	m_blue;
		
		public RGBColor(int red, int green, int blue)
		{
			m_red	= red;
			m_green = green;
			m_blue  = blue;
		}
		
		public static RGBColor White()
		{
			return(new RGBColor(255, 255, 255));
		}
		
		public static RGBColor Red()
		{
			return(new RGBColor(255, 0, 0));
		}
		
		public static RGBColor Yellow()
		{
			return(new RGBColor(255, 255, 0));
		}
		
		public static RGBColor Black()
		{
			return(new RGBColor(0, 0, 0));
		}
	}
	
	private static int m_nStrings = 0;
	private int m_stringNo = -1;
	private static ArduinoConnection m_arduino;
	private RGBColor[] m_colors;
	private RGBColor[] m_lastColors;
	private boolean m_update = false;
	private String m_logFile = null;
	
	private void SetupLog(String name)
	{
		// m_logFile = name;
		
		// Logger.SetLogFile(m_logFile, m_logFile);
    	// ArduinoConnection.GetInstance().SetCommandLogFile(m_logFile);
    	// Logger.Log(m_logFile, 1, "LED Log start");
	}
	
	static public Future<?> Schedule(Runnable task, int time)
	{
		return(m_scheduler.scheduleAtFixedRate(task, 0, time, TimeUnit.MILLISECONDS));
	}
	
	public void SendCommand(String command)
	{
//		Logger.Log("LED5050String", 1, "Command: " + command);
		// if (m_logFile != null)
		// {
		// 	Logger.Log(m_logFile, 1, String.format("l:%d:", System.currentTimeMillis()) + command);
		// }
		m_nCharsSent += command.length();
		
		if (m_arduino == null)
		{
			m_arduino = new ArduinoConnection();
		}

		m_arduino.SendCommand(command);
	}
	
//	static int[] k_colorMap = new int[] { 0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e, 0x20, 0x40, 0x60, 0x80, 0xa0, 0xc0, 0xe0, 0xff };
	static int[] k_colorMap = new int[] { 0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xa0, 0xb0, 0xc0, 0xd0, 0xe0, 0xf0 };
	
	private void TestColors()
	{
		int nLeds = m_colors.length;
		
		if (nLeds >= 16*3)
		{
			
			for (int i = 0 ; (i < 16) ; i++)
			{
				m_colors[i] = new RGBColor(k_colorMap[i], 0, 0);
				m_colors[i+16] = new RGBColor(0, k_colorMap[i], 0);
				m_colors[i+32] = new RGBColor(0, 0, k_colorMap[i]);
			}
		}
	}
	
	private static LED5050String[] m_strings = null;
	private static int m_nCharsSent = 0;
	
	/** 
	 *  @param pin - Specifies the pin to which the string is attached
	 *  @param nLeds - Specifies the number of lights on the string
	 */
	public LED5050String(int pin, int nLeds)
	{
		int maxStrings = 4;	//m_arduino.GetMaxLEDStrings();
		
		if (m_nStrings < maxStrings)
		{
			if (m_nStrings == 0)
			{
				m_strings = new LED5050String[maxStrings];
				
				Schedule(
			    		new Runnable() 
					    {
					        @Override
					        public void run() 
					        {
//					        	Logger.Log("LED5050String", 1, "Update: " + m_update);
					        	if (m_update)
					        	{
//						        	Logger.Log("LED5050String", 1, "Updating...");
						        	
						        	synchronized(m_lock)
						        	{
//						        		Logger.Log("LED5050String", 1, "Aquired lock");
						        		m_nCharsSent = 0;
						        		for (int i = 0 ; i < m_nStrings ; i++)
						        		{
//						        			Logger.Log("LED5050String", 1, String.format("i=%d", i));
						        			m_strings[i].SendChangedColors();
						        		}
						        		
						        		SendCommand("l0u");
						        		
						        		m_update = false;
						        	}
						        	
//						        	Logger.Log("LED5050String",  1, "Lock released");
						        	
						        	int sleepTime = (int) (0.1 * m_nCharsSent)+10;	// Time required to send chars at 115200 bits/sec
						        	
					        		if (sleepTime > 0)
					        		{
//					        			Logger.Log("LED5050String", 1, String.format("Sleep for %d", sleepTime));
					        			try {
//					        				m_arduino.flush();
											Thread.sleep(sleepTime);
										} catch (InterruptedException e) {
											// TODO Auto-generated catch block
											e.printStackTrace();
										}
					        		}
					        	}
					        }
					    }, k_updateTime);		
				
			}
			
			m_stringNo = m_nStrings;
			m_colors = new RGBColor[nLeds];
			m_lastColors = new RGBColor[nLeds];
			
			for (int i = 0 ; i < nLeds ; i++)
			{
				m_colors[i] = new RGBColor(0, 0, 0);
				m_lastColors[i] = new RGBColor(-1, -1, -1);
			}
			TestColors();
			
//			Logger.Log("LED505String", 1, String.format("l%dc %d %d", m_stringNo, pin, nLeds));
			
//			SetupLog(String.format("LEDLog%d-", m_stringNo));
			
			synchronized(m_lock)
			{
				m_strings[m_stringNo] = this;
				
				SendCommand(String.format("l%dc %d %d", m_stringNo, pin, nLeds));
				m_nStrings++;
				m_update = true;
			}
			
//			SendColors(0, nLeds - 1);
//			SendCommand("l0u");
//			Update();
		}
	}
	
	/** 
	 *  @return Returns the length of the string
	 */
	public int GetLength()
	{
		return(m_colors.length);
	}
	
	private void SendColors(int pos, int nColors)
	{
		synchronized(m_lock)
		{
			while (nColors > 0)
			{
				int n = (nColors > 10) ? 10 : nColors;
				String command = String.format("l%ds %d ", m_stringNo, pos);
				
				for (int i = 0 ; i < n ; i++, pos++)
				{
//					command += String.format("%02x%02x%02x", 
//												m_colors[pos].m_red & 0xff, 
//												m_colors[pos].m_green & 0xff, 
//												m_colors[pos].m_blue & 0xff);
					command += String.format("%01x%01x%01x", 
													(m_colors[pos].m_red >> 4) & 0xf, 
													(m_colors[pos].m_green >> 4) & 0xf, 
													(m_colors[pos].m_blue >> 4) & 0xf);
					
				}
				
				nColors -= n;
				
				SendCommand(command);
			}
		}
	}
	
	/** 
	 *  @param colors - An array which specifies the color for each of the lights on the string.
	 *  				The length of this array should equal the length of the light string.
	 */
	public boolean SetColors(RGBColor[] colors)
	{
		if (colors.length != m_colors.length)
		{
			// Logger.Log("LED5050String", 4, String.format("SetColors: Invalid string length: %d", colors.length));
			
			return(false);
		}
		
		synchronized (m_lock)
		{
			for (int i = 0 ; i < m_colors.length ; i++)
			{
				m_colors[i].m_red = colors[i].m_red;
				m_colors[i].m_green = colors[i].m_green;
				m_colors[i].m_blue = colors[i].m_blue;
			}
			m_update = true;
		}
		
		return(true);
	}
	
	private void SendChangedColors()
	{
		int	first = -1;
		int last  = -1;
		
		for (int i = 0 ; i < m_colors.length ; i++)
		{
			if ((m_colors[i].m_red != m_lastColors[i].m_red)	||
				(m_colors[i].m_green != m_lastColors[i].m_green)	||
				(m_colors[i].m_blue != m_lastColors[i].m_blue))
			{
				if (first < 0)
				{
					first = i;
				}
				last = i;
				
				m_lastColors[i].m_red = m_colors[i].m_red;
				m_lastColors[i].m_green = m_colors[i].m_green;
				m_lastColors[i].m_blue = m_colors[i].m_blue;
			}
		}
		
		if (first >= 0)
		{
			int nColors = last - first + 1;
			
			m_nCharsSent += nColors;
			
			SendColors(first, nColors);
//			SendCommand("l0u");
		}
	}
	
	/** 
	 *  Causes all of the strings to be updated
	 */
	public void Update()
	{
//		Logger.Log("LED5050String", 1, String.format("l0u"));		
//		m_arduino.SendCommand(String.format("l0u"));	
		
		synchronized(m_lock)
		{
			m_update = true;
		}
	}
}