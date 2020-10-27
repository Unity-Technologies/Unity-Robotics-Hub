using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using RosMessageGeneration;
using UnityEngine;

/// <summary>
/// 	Meant to be inherited by a Unity script this class provides all the functionality
/// 	to start a TCP server and handle incoming connections.
///
/// 	Specifically made to accept incoming connections from
/// 		https://github.com/Unity-Technologies/Robotics-Tutorials/tree/master/catkin_ws/src/tcp_endpoint
///
///
/// 	All incoming messages are expected to come in the format of:
/// 		first four bytes: int32 of the length of following string value
/// 		next N bytes determined from previous four bytes: ROS topic name as a string
/// 		next four bytes: int32 of the length of the remaining bytes for the ROS Message
/// 		last N bytes determined from previous four bytes: ROS Message variables
/// </summary>
public class RosSubscriber : MonoBehaviour
{
	public int hostPort = 5005;
	
    private TcpConnector tcpCon;
    
    static object _lock = new object(); // sync lock 
    static List<Task> _connections = new List<Task>(); // pending connections

	/// <summary>
	///		Given a network stream and empty Message class, read all available bytes from the network stream
	/// 	and use the passed Message class to deserialize the read bytes and update the class' property values
	/// </summary>
	/// <param name="networkStream"></param> Input stream to read from
	/// <param name="customMessage"></param> Message type used to deserialize bytes from input stream
	/// <returns> Deserialized message class </returns>
    public static Message ReadMessage(NetworkStream networkStream, Message customMessage)
    {
	    try {
		    if(networkStream.CanRead){
			    int offset = 0;
			    
			    // Get first bytes to determine length of topic name
			    byte[] rawTopicBytes = new byte[4];
			    networkStream.Read(rawTopicBytes, 0, rawTopicBytes.Length);
			    offset += 4;
			    int topicLength = BitConverter.ToInt32(rawTopicBytes, 0);

			    // Read and convert topic name
			    byte[] topicNameBytes = new byte[topicLength];
			    networkStream.Read(topicNameBytes, 0, topicNameBytes.Length);
			    offset += topicNameBytes.Length;
			    string topicName = Encoding.ASCII.GetString(topicNameBytes, 0, topicLength);
			    // TODO: use topic name to confirm proper received location

			    byte[] full_message_size_bytes = new byte[4];
			    networkStream.Read(full_message_size_bytes, 0, full_message_size_bytes.Length);
			    offset += 4;
			    int full_message_size = BitConverter.ToInt32(full_message_size_bytes, 0);

			    var readBuffer = new byte[full_message_size];
			    int numberOfBytesRead = 0;
                    
			    while(networkStream.DataAvailable && numberOfBytesRead < full_message_size)
			    {
				    var bytesRead = networkStream.Read(readBuffer, 0, readBuffer.Length);
				    offset += bytesRead;
				    numberOfBytesRead += bytesRead;
			    }

			    customMessage.Deserialize(readBuffer, 0);
			    return customMessage;
		    }
	    }
	    catch (Exception e){
		    Debug.LogError("Exception raised!! " + e);
	    }

	    return null;
    }

    /// <summary>
    /// 	Function is meant to be overridden by inheriting classes to specify how to handle read messages.
    /// </summary>
    /// <param name="tcpClient"></param> TcpClient to read byte stream from.
    protected virtual async Task HandleConnectionAsync(TcpClient tcpClient)
    {
	    await Task.Yield();
	    // continue asynchronously on another threads

	    using (var networkStream = tcpClient.GetStream())
	    {
		    Debug.Log("Message Received."); 
	    }
    }
    
    /// <summary>
    /// 	Handles multiple connections and locks.
    /// </summary>
    /// <param name="tcpClient"></param> TcpClient to read byte stream from.
    private async Task StartHandleConnectionAsync(TcpClient tcpClient)
    {
	    // start the new connection task
	    var connectionTask = HandleConnectionAsync(tcpClient);

	    // add it to the list of pending task 
	    lock (_lock)
		    _connections.Add(connectionTask);

	    // catch all errors of HandleConnectionAsync
	    try
	    {
		    await connectionTask;
		    // we may be on another thread after "await"
	    }
	    catch (Exception ex)
	    {
		    Debug.LogError(ex.ToString());
	    }
	    finally
	    {
		    // remove pending task
		    lock (_lock)
			    _connections.Remove(connectionTask);
	    }
    }
    
    /// <summary>
    /// 	Creates the TcpListener with the appropriate functions to handle incoming connections.
    /// </summary>
    /// <param name="hostPort"></param> Port on local machine to listen for incoming connections
    protected async void StartMessageServer(int hostPort)
    {
	    while (true)
	    {
		    try {
			    var tcpListener = TcpListener.Create(hostPort);  
			    tcpListener.Start();

			    while (true)   //we wait for a connection
			    {
				    var tcpClient = await tcpListener.AcceptTcpClientAsync();

				    var task = StartHandleConnectionAsync(tcpClient);
				    // if already faulted, re-throw any error on the calling context
				    if (task.IsFaulted)
					    await task;
			    }
		    }
		    catch (Exception e){
			    Debug.LogError("Exception raised!! " + e);
		    }
	    }
    }

}
