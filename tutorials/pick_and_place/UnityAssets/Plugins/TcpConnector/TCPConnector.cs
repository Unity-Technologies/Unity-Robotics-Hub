using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using RosMessageGeneration;
using Debug = UnityEngine.Debug;

/// <summary>
///
/// TCPConnector to facilitate communication with TCP endpoint running as a ROS node
/// 	https://github.com/Unity-Technologies/Robotics-Tutorials/tree/master/catkin_ws/src/tcp_endpoint
///
/// NOTE: Limited to mimic ROS publisher and service
/// Can publish messages to topics and send messages to services while receiving the response message(s).
/// 
/// </summary>
public class TcpConnector 
{
	private string hostName;
	private int hostPort;
	private int networkTimeout = 2000;
	private int bufferSize = 1024;
	private static object _lock = new object(); // sync lock
	private static List<Task> _connections = new List<Task>(); // pending connections
	private int dataAvailableMaxPoll;
	private int dataAvailablePollSleep;

    public Dictionary<string, Message> topicMessages;
    
    /// <summary>
    /// 	Class used to communicate with TCP endpoint running as ROS node.
    /// </summary>
    /// <param name="hostName"></param> The host name of the TCP endpoint running as ROS node
    /// <param name="hostPort"></param> The host port of the TCP endpoint running as ROS node
    /// <param name="networkTimeout"></param> Network timeout for TCP connection
    /// <param name="async"></param> Should messages be sent asynchronously
    /// <param name="topicMessages"></param> Dictionary of topic or service name to it's corresponding ROS message type
    public TcpConnector(string hostName, int hostPort, int networkTimeout = 2000, bool async = false,
	    Dictionary<string, Message> topicMessages = null, int serviceResponseRetry = 3, int serviceResponseSleep = 1000)
    {
        this.hostName = hostName;
        this.hostPort = hostPort;
        this.networkTimeout = networkTimeout;
        this.topicMessages = topicMessages;
        this.dataAvailableMaxPoll = serviceResponseRetry;
        this.dataAvailablePollSleep = serviceResponseSleep;
    }

		/// <summary>
        ///    Given some input values, fill a byte array in the desired format to use with
        ///     https://github.com/Unity-Technologies/Robotics-Tutorials/tree/master/catkin_ws/src/tcp_endpoint
        ///
        /// 	All messages are expected to come in the format of:
        /// 		first four bytes: int32 of the length of following string value
        /// 		next N bytes determined from previous four bytes: ROS topic name as a string
        /// 		next four bytes: int32 of the length of the remaining bytes for the ROS Message
        /// 		last N bytes determined from previous four bytes: ROS Message variables
        /// </summary>
        /// <param name="offset"></param> Index of where to start writing output data
        /// <param name="serviceName"></param> The name of the ROS service or topic that the message data is meant for
        /// <param name="fullMessageSizeBytes"></param> The full size of the already serialized message in bytes
        /// <param name="messageToSend"></param> The serialized ROS message to send to ROS network
        /// <returns></returns>
        public int GetPrefixBytes(int offset, byte[] serviceName, byte[] fullMessageSizeBytes, byte[] messagBuffer)
        {
            // Service Name bytes
            System.Buffer.BlockCopy(serviceName, 0, messagBuffer, 0, serviceName.Length);
            offset += serviceName.Length;

            // Full Message size bytes
            System.Buffer.BlockCopy(fullMessageSizeBytes, 0, messagBuffer, offset, fullMessageSizeBytes.Length);
            offset += fullMessageSizeBytes.Length;

            return offset;
        }

        /// <summary>
        ///    Serialize a ROS message in the expected format of
        ///     https://github.com/Unity-Technologies/Robotics-Tutorials/tree/master/catkin_ws/src/tcp_endpoint
        ///
        /// 	All messages are expected to come in the format of:
        /// 		first four bytes: int32 of the length of following string value
        /// 		next N bytes determined from previous four bytes: ROS topic name as a string
        /// 		next four bytes: int32 of the length of the remaining bytes for the ROS Message
        /// 		last N bytes determined from previous four bytes: ROS Message variables
        /// </summary>
        /// <param name="topicServiceName"></param> The ROS topic or service name that is receiving the messsage
        /// <param name="messsage"></param> The ROS message to send to a ROS publisher or service
        /// <returns> byte array with serialized ROS message in appropriate format</returns>
        public byte[] GetMessageBytes(string topicServiceName, Message message)
        {
            byte[] topicName = message.SerializeString(topicServiceName);
            byte[] bytesMsg = message.Serialize();
            byte[] fullMessageSizeBytes = BitConverter.GetBytes(bytesMsg.Length);

            byte[] messageBuffer = new byte[topicName.Length + fullMessageSizeBytes.Length + bytesMsg.Length];
            // Copy topic name and message size in bytes to message buffer
            int offset = GetPrefixBytes(0, topicName, fullMessageSizeBytes, messageBuffer);
            // ROS message bytes
            System.Buffer.BlockCopy(bytesMsg, 0, messageBuffer, offset, bytesMsg.Length);

            return messageBuffer;
        }

    /// <summary>
    /// 	Connects to TCP endpoint ROS node and publishes a message to a particular topic.
    /// </summary>
    /// <param name="rosTopicName">The ROS topic name to publish the message to.</param>
    /// <param name="messageData">ROS message to publish to a topic</param>
    public void SendMessage(string rosTopicName, Message message)
    {
	    try
	    {
		    // Serialize the message in topic name, message size, and message bytes format
		    byte[] messageBytes = GetMessageBytes(rosTopicName, message);

		    TcpClient client = new TcpClient();
		    client.Connect(hostName, hostPort);

		    NetworkStream networkStream = client.GetStream();
		    networkStream.ReadTimeout = networkTimeout;

		    networkStream.Write(messageBytes, 0, messageBytes.Length);

		    if (client.Connected)
			    client.Close();
	    }
	    catch (NullReferenceException e)
	    {
		    Debug.LogError("TCPConnector.SendMessage Null Reference Exception: " + e);
	    }
        catch (Exception e)
        {
            Debug.LogError("TCPConnector Exception: " + e);
        }
    }

    /// <summary>
	/// 	Connects to TCP endpoint ROS node and writes a message to a particular service.
	/// </summary>
	/// <param name="rosServiceName">ROS Service name to handle the request</param>
	/// <param name="serviceRequest">Service Request Message to send to ROS service in bytes</param>
	/// <param name="serviceResponse">CustomMessage class of the response</param>
	/// <returns></returns>
    public Message SendServiceMessage(string rosServiceName, Message serviceRequest, Message serviceResponse)
    {
	    // Serialize the message in service name, message size, and message bytes format
	    byte[] messageBytes = GetMessageBytes(rosServiceName, serviceRequest);

	    TcpClient client = new TcpClient();
        client.Connect(hostName, hostPort);
        
        NetworkStream networkStream = client.GetStream();
        networkStream.ReadTimeout = networkTimeout;

        // Send the message
        try
        {
	        networkStream.Write(messageBytes, 0, messageBytes.Length);
        }
        catch (Exception e)
        {
	        Debug.LogError("SocketException: " + e);
        }

        int numberOfBytesRead = 0;
        while (numberOfBytesRead == 0)
    	{
    		try {
	            if(networkStream.CanRead)
	            {
		            // Poll every 1 second(s) for available data on the stream
		            int attempts = 0;
		            while (!networkStream.DataAvailable && attempts <= this.dataAvailableMaxPoll)
		            {
			            if (attempts == this.dataAvailableMaxPoll)
			            {
				            Debug.LogError("No data available on network stream after " + dataAvailableMaxPoll + " attempts.");
				            return null;
			            }
			            attempts++;
			            Thread.Sleep(this.dataAvailablePollSleep);
		            }

		            // Get first bytes to determine length of service name
    				byte[] rawServiceBytes = new byte[4];
    				networkStream.Read(rawServiceBytes, 0, rawServiceBytes.Length);
					int topicLength = BitConverter.ToInt32(rawServiceBytes, 0);

					// Create container and read service name from network stream
                    byte[] serviceNameBytes = new byte[topicLength];
                    networkStream.Read(serviceNameBytes, 0, serviceNameBytes.Length);
                    string serviceName = Encoding.ASCII.GetString(serviceNameBytes, 0, topicLength);

                    // Get leading bytes to determine length of remaining full message
                    byte[] full_message_size_bytes = new byte[4];
                    networkStream.Read(full_message_size_bytes, 0, full_message_size_bytes.Length);
                    int full_message_size = BitConverter.ToInt32(full_message_size_bytes, 0);

                    // Create container and read message from network stream
                    byte[] readBuffer = new byte[full_message_size];
                    while(networkStream.DataAvailable && numberOfBytesRead < full_message_size)
                    {
	                    var readBytes = networkStream.Read(readBuffer, 0, readBuffer.Length);
	                    numberOfBytesRead += readBytes;
                    }

                    serviceResponse.Deserialize(readBuffer, 0);
	            }
	            else{
    				Debug.LogError("Sorry, you cannot read from this NetworkStream.");
                    return null;
	            }

    		}
    		catch (Exception e){
                Debug.LogError("Exception raised!! " + e);
                return null;
            }
    	}

        if (client.Connected)
	        client.Close();

        return serviceResponse;
    }
}

