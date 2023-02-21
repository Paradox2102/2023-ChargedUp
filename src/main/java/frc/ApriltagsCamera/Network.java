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

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintStream;
import java.net.ServerSocket;
import java.net.Socket;


/**
 * 
 * @author John Gaby
 * 
 * @brief The Network class handles network connections.
 * 
 * This class handles network connections. You can choose to either connect to a specified IP address
 * or to listen for incoming connections of a specified port.
 * 
 * In either case, a two way connection is established. The receiver end of the connection is handled
 * via a separate thread and there is a callback provided when data is received.
 * 
 * The transmit side runs in whatever thread the calling function is using.
 *
 */
public class Network 
{
	private String 			m_host;
	private int				m_port;
	private Receiver		m_receiver;
	private NetworkReceiver	m_networkReceiver;
	private int				m_retryTime;
	private PrintStream 	m_printStream = null;
	private Object			m_lock = new Object();
	
	private ServerSocket m_serverSocket = null;
	private Socket m_clientSocket = null;
	private InputStream m_inputStream = null;
	private OutputStream m_outputStream = null;


	/**
	 * 
	 * @author John Gaby
	 * 
	 * @brief The NetworkReceiver interface provides a callback for received data on a network connection.
	 *
	 */
	public interface NetworkReceiver
	{
		/**
		 * When data is received over the network connection, it is treated as a string. When the newline character
		 * is received, the entire string representing this line is sent to the data handler via this function
		 * 
		 * @param command - Specifies the data received
		 * 
		 */
		public void processData(String command);
		public void connected();
		public void disconnected();
	}
	

	/**
	 * This function attempts to connect to the specified host. If the connection fails, it
	 * will retry the connection. If the connection succeeds, a thread will be started that
	 * will retrieve any data sent over the connection and pass that data on to a handler
	 * via the <strong>NetworkReceiver</strong> interface. 
	 * 
	 * @param receiver - Specifies class that will receive data from the connection
	 * @param host - Specifies the host IP address
	 * @param port - Specifies the host port number
	 * @param retryTime - Specifies the amount of time in milliseconds to retry failed connections
	 * 
	 */
	public void connect(NetworkReceiver receiver, String host, int port, int retryTime)
	{
		m_host				= host;
		m_port				= port;
		m_networkReceiver	= receiver;
		m_retryTime			= retryTime;
		m_receiver			= new Receiver();
		
		new Thread(m_receiver).start();
	}
	
	/**
	 * This function attempts to connect to the specified host. If the connection fails, it
	 * will retry the connection. If the connection succeeds, a thread will be started that
	 * will retrieve any data sent over the connection and pass that data on to a handler
	 * via the <strong>NetworkReceiver</strong> interface.
	 * 
	 *  It uses the default retry time of 5000 ms.
	 * 
	 * @param receiver - Specifies class that will receive data from the connection
	 * @param host - Specifies the host IP address
	 * @param port - Specifies the host port number
	 * 
	 */
	public void connect(NetworkReceiver receiver, String host, int port)
	{
		connect(receiver, host, port, 5000);
	}
	
	/**
	 * This function sends the specified string via the connection if the connection is open.
	 * 
	 * @param message - Specifies message to be sent
	 * 
	 */
	public void sendMessage(String message)
	{
		if (m_networkReceiver != null)
		{
			synchronized(m_lock)
			{
				if (m_printStream != null)
				{
					m_printStream.println(message);
				}
			}
		}
	}
	
	/**
	 * This function will start a new thread which will listen on
	 * the specified port for an incoming connection. Once the connection
	 * is made, it will retrieve any data sent over the connection (using the
	 * created thread) and pass that data on to a handler via the <strong>NetworkReceiver</strong> interface.
	 * 
	 * @param receiver - Specifies class that will receive data from the connection
	 * @param port - Specifies the port to listen on
	 * 
	 */
	public void listen(NetworkReceiver receiver, int port)
	{
		m_host				= null;
		m_port				= port;
		m_networkReceiver	= receiver;
		m_receiver			= new Receiver();
		
		new Thread(m_receiver).start();
	}
	
	/**
	 * This function closes the connection.
	 *
	 */
	public void closeConnection()
	{
		System.out.println("CloseConnection");
		
		m_networkReceiver.disconnected();
		
		synchronized(m_lock)
		{
			if (m_printStream != null)
			{
				m_printStream.close();
				m_printStream	= null;
			}
			
			if (m_outputStream != null)
			{
				try {
					m_outputStream.close();
				} catch (IOException e) {
				}
				m_outputStream = null;
			}
			
			if (m_inputStream != null)
			{
				try {
					m_inputStream.close();
				} catch (IOException e) {
				}
				m_inputStream = null;
			}
			
			if (m_clientSocket != null)
			{
				try {
					m_clientSocket.close();
				} catch (IOException e) {
				}
				m_clientSocket = null;
			}
			
			if (m_serverSocket != null)
			{
				try {
					m_serverSocket.close();
				} catch (IOException e) {
				}
				m_serverSocket = null;
			}
		}
	}
	
	private class Receiver implements Runnable
	{
		private void runClient()
		{
			Logger.log("Network", 1, String.format("%s:%d: Client Thread started", m_host, m_port));
			
			while (true)
			{
				Logger.log("Network", 1, String.format("Connecting to %s:%d", m_host, m_port));
				
				try 
				{
					Socket clientSocket = new Socket(m_host, m_port);
					InputStream inputStream = clientSocket.getInputStream();
					OutputStream outputStream = clientSocket.getOutputStream();
					PrintStream printStream = new PrintStream(outputStream);

					synchronized(m_lock)
					{
						m_clientSocket = clientSocket;
						m_inputStream = inputStream;
						m_outputStream = outputStream;
						m_printStream = printStream;
					}	

					String command = "";
					
					Logger.log("Network", 1, String.format("Connected to %s:%d", m_host, m_port));
					
					m_networkReceiver.connected();
					
					while (true)
					{
						int ch = m_inputStream.read();
						
						if (ch < 0)
						{
							Logger.log("Network", 2, "Connection lost");
						
							break;
						}
						
						if (ch == '\n')
						{
							if (command.length() >= 1)
							{
								m_networkReceiver.processData(command);
							}
							
							command	= "";
							
						}
						else if (ch >= 0)
						{
							command += (char) ch;
						}
						else
						{
							Thread.sleep(5);
						}
					}
					
				}
				catch (Exception ex)
				{
					Logger.log("Network",  3, "Receiver exception: " + ex);
				}
				
				closeConnection();
				
				if (m_retryTime == 0)
				{
					break;
				}
				
				try {
					Thread.sleep(m_retryTime);
				} catch (InterruptedException e) {

				}
			}
			
			Logger.log("Network", 1, String.format("%s:%d: Thread exit", m_host, m_port));
		}
		
		private void runHost()
		{
			Logger.log("Network", 1, String.format("Host Thread started on port %d", m_port));
			
			while (true)
			{
				Logger.log("Network", 1, String.format("Waiting for connection on port %d", m_port));
				
				try
				{
					m_serverSocket = new ServerSocket(m_port);
					m_clientSocket = m_serverSocket.accept();
					synchronized(m_lock)
					{
						m_inputStream = m_clientSocket.getInputStream();
						m_outputStream = m_clientSocket.getOutputStream();
						m_printStream = new PrintStream(m_outputStream);
					}

					int ch;
					String	command	= "";
					
					Logger.log("Network", 2, "Host connected");
					
					m_networkReceiver.connected();
					
					do
					{
						ch = m_inputStream.read();
						
						if (ch == '\n')
						{
							m_networkReceiver.processData(command);
							command	= "";
							
						}
						else if (ch != -1)
						{
							command += (char) ch;
						}
					} while (ch > 0);
				}
				catch (Exception ex)
				{
					Logger.log("Network", 3, "Host network error: " + ex);
					
					closeConnection();

					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {

					}
				}
			}
		}
		
		@Override
		public void run() 
		{
			if (m_host == null)
			{
				runHost();
			}
			else
			{
				runClient();
			}
		}
	}
}
