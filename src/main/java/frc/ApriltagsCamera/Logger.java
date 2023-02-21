/*
 *	  Copyright (C) 2022  John H. Gaby
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
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import frc.ApriltagsCamera.Network.NetworkReceiver;

/**
 * 
 * @author John Gaby
 * 
 * @brief The Logger class provides a mechanism to log messages
 * 
 * This class provides a mechanism to log messages to either the console or a file or both.
 * The messages are tagged by a name and a logging level.  There is a global setting for the
 * minimum logging level of messages.  Messages with a logging level less than the minimum will
 * not be logged.  The default global logging level is -999 (i.e. everything is logged)
 * 
 * In addition, each specific tag can have an independent minimum logging level which controls
 * whether messages of that type are logged.  The default logging level for specific tags is 0.
 * 
 * By default messages are logged to the console.  However you can specify that the messages be
 * logged to a file, or both a file and the console.  You can set a global file which will log all
 * messages that do not have their own file specified.  You can also specify a separate file for
 * each tag.
 * 
 */

public class Logger 
{
	private static Object 		m_lock = new Object();
	private final static String m_traceName = "trace";
	private final static String m_logDir	= "/home/pi/logs/";
	private static PrintWriter	m_file		= null;
	private static int			m_level		= -999;			// By default, use logging level based on the type
	private static long			m_startTime	= System.currentTimeMillis();
	private static Network		m_network	= null;
	
	static private PrintWriter createLog(String name)
	{
		DateFormat 	dateFormat 	= new SimpleDateFormat("MM-dd-HH-mm-ss");
		Date 		date 		= new Date();
		String		path		= m_logDir + name + dateFormat.format(date) + ".txt";
		PrintWriter	file		= null;
		
		try 
		{
			file	= new PrintWriter(path);
		} 
		catch (FileNotFoundException e) 
		{
			e.printStackTrace();
		}	
		
		return(file);
	}
	
	static private class LogType
	{
		private String		m_name			= null;
		private int			m_level			= 0;
		private PrintWriter	m_file			= null;
		private boolean		m_logToConsole	= true;
		private boolean		m_logTime		= false;
//		private long		m_startTime		= System.currentTimeMillis();
		
		public LogType(String name)
		{
			m_name	= name;
			
			if (m_name.equals(m_traceName))
			{
				m_logTime	= true;
			}
		}
		
		public void closeLog()
		{
			if (m_file != null);
			{
				m_file.close();
				m_file	= null;
			}
		}
		
		public void createLog(String name, boolean logTime, boolean logToConsole)
		{
			m_logToConsole	= logToConsole;
			m_logTime		= logTime;
			
			if (m_file != null)
			{
				m_file.close();
			}
			
			m_file	= Logger.createLog(name);
		}
		
/*		public long GetElapsedTime()
		{
			return(System.currentTimeMillis() - m_startTime);
		}*/
		
		public void setLogLevel(int level)
		{
			m_level	= level;
		}
	}
	
	static private ArrayList<LogType>	m_types = new ArrayList<LogType>();
	
	static private LogType findType(String tag)
	{
		if (tag	== null)
		{
			tag	= m_traceName;
		}
		
		for (LogType type : m_types)
		{
			if (type.m_name.equals(tag))
			{
				return(type);
			}
		}
		
		/*
		 * Tag not found, add it
		 */
		Logger.LogType	newType	= new Logger.LogType(tag);
		m_types.add(newType);
		
		return(newType);
	}
	
	/**
	 * 
	 * @param tag - Specifies the tag for this message.
	 * @param level - Specifies the logging level.  If this value is less than the current minimum level, this message will not be logged.
	 * @param message - Specifies the message to log.
	 * @param noLogToConsole - Specifies the this message should NOT be logged to che console.
	 */
	static public void log(String tag, int level, String message, boolean noLogToConsole)
	{
		synchronized(m_lock)
		{
			if (level >= m_level)
			{
				LogType	type	= findType(tag);
				
				if (level >= type.m_level)
				{
					PrintWriter	file;
					boolean logTime;
					
					if (type.m_file == null)
					{
						file	= m_file;
						logTime	= true;			// Always log the time to the master file
					}
					else
					{
						file	= type.m_file;
						logTime	= type.m_logTime;
					}
					
					if (file != null)
					{
//						System.out.println("Log: logTime = " + logTime);
						if (logTime)
						{
							file.print(getElapsedTime() + ":" + type.m_name + "(" + level + "):");
						}
						
						file.println(message + "\r");
						file.flush();
					}
					
					if ((file == null) || (type.m_logToConsole && !noLogToConsole))
					{
						String log = "" + getElapsedTime() + ":" + type.m_name + "(" + level + "):" + message;
						System.out.println(log);
						
						if (m_network != null)
						{
//							String msg = log + "\r\n" + "xxx";
//							System.out.print("network: " + msg);
							m_network.sendMessage(log);
						}
						
//						System.out.print(GetElapsedTime() + ":");	// Always log time to console
//						System.out.println(type.m_name + "(" + level + "):" + message);
					}
				}
			}
		}
	}
	
	static public void log(String tag, int level, String message)
	{
		log(tag, level, message, false);
	}
	/**
	 * 
	 * @param level - Specifies the global logging level.  No messages less than this level will be logged, regardless of the logging level for the log type.
	 */
	static public void setLogLevel(int level)
	{
		synchronized(m_lock)
		{
			m_level	= level;
		}
	}

	/**
	 * 
	 * @param tag - Specifies the tag for the log type for which the level is to be set.
	 * @param level - Specifies the new logging level.  For messages that are tagged with this tag, if the log level of the message is less than this value, the message will not be logged.
	 */
	static public void setLogLevel(String tag, int level)
	{
		synchronized(m_lock)
		{
			findType(tag).setLogLevel(level);
		}
	}
	
	/**
	 * 
	 * @param name - Specifies a file name.  All logs of messages that do not have their own individual log files will be written to this file
	 */
	static public void setLogFile(String name)
	{
		synchronized(m_lock)
		{
			if (m_file != null)
			{
				m_file.close();
			}
			
			m_file	= createLog(name);
		}
	}
	
	/**
	 * 
	 * @param tag - Specifies the tag that is to be logged to a file.
	 * @param name - Specifies a file name.  All logs that are tagged with this value will be logged to this file
	 * 
	 * Calls <strong>SetLogFile(tag, name, false)</strong>
	 * 
	 */
	static public void setLogFile(String tag, String name)
	{
		setLogFile(tag, name, false, false);
	}
	
	/**
	 * 
	 * @param tag - Specifies the tag that is to be logged to a file.
	 * @param name - Specifies a file name.  All logs that are tagged with this value will be logged to this file
	 * @param logTime - If <strong>true</strong>, include the current time stamp for each logged line.
	 * 
	 * Calls <strong>SetLogFile(tag, name, logTime, false)</strong>
	 * 
	 */
	static public void setLogFile(String tag, String name, boolean logTime)
	{
		setLogFile(tag, name, logTime, false);
	}
	
	/**
	 * 
	 * @param tag - Specifies the tag that is to be logged to a file.
	 * @param name - Specifies a file name.  All logs that are tagged with this value will be logged to this file
	 * @param logTime - If <strong>true</strong>, include the current time stamp for each logged line.
	 * @param logToConsole - If <strong>true</strong>, log messages to the console as well as the file
	 */
	static public void setLogFile(String tag, String name, boolean logTime, boolean logToConsole)
	{
		synchronized(m_lock)
		{
			findType(tag).createLog(name, logTime, logToConsole);
		}
	}
	
	/**
	 * Close the master log file
	 */
	static public void closeLogFile()
	{
		synchronized(m_lock)
		{
			if (m_file != null)
			{
				m_file.close();
			}
		}
	}

	/**
	 * 
	 * @param tag - Close the log file associated with messages with this tag.
	 * 
	 */
	static public void closeLogFile(String tag)
	{
		synchronized(m_lock)
		{
			findType(tag).closeLog();
		}
	}
	
	/**
	 * Close all log files.
	 * 
	 */
	static public void closeAllLogFiles()
	{
		closeLogFile();
		
		synchronized(m_lock)
		{
			for (LogType type : m_types)
			{
				type.closeLog();
			}
		}
	}
	
	/**
	 * 
	 * @return Returns the time elapsed since the last call to <strong>ResetElapsedTime()</strong>
	 */
	static public long getElapsedTime()
	{
		synchronized(m_lock)
		{
			return(System.currentTimeMillis() - m_startTime);
		}
	}
	
	/**
	 * 
	 * Resets the elapsed time to zero
	 */
	static public void resetElapsedTime()
	{
		synchronized(m_lock)
		{
			m_startTime	= System.currentTimeMillis();
		}
	}
	
	static private class Receiver implements NetworkReceiver
	{
		@Override
		public void processData(String command) 
		{
		}

		@Override
		public void connected() {
		}

		@Override
		public void disconnected() {
		}
	}
	
	/**
	 * 
	 * Starts a network server that can be contacted to receive the log strings.
	 * The server listens on port 5810.
	 */
	static public void startLoggingServer()
	{
		if (m_network == null)
		{
			m_network	= new Network();
			
			m_network.listen(new Receiver(), 5810);
		}
	}
}
