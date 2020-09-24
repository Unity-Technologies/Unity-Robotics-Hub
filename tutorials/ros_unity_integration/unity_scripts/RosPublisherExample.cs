using RosMessageTypes.RoboticsDemo;
using UnityEngine;
using Random = UnityEngine.Random;

/// <summary>
///
/// </summary>
public class RosPublisherExample : MonoBehaviour
{
    // Create a new ROS
    private TcpConnector tcpCon;

    // Variables required for ROS communication
    public string topicName = "pos_rot";
    public string hostName = "192.168.1.116";
    public int hostPort = 10000;

    // The game object
    public GameObject cube;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // Instantiate the connector with ROS host name and port.
        tcpCon = new TcpConnector(hostName, hostPort);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 0.5f)
        {
            var xAngle = Random.Range(-100.0f, 100.0f);
            var yAngle = Random.Range(-100.0f, 100.0f);
            var zAngle = Random.Range(-100.0f, 100.0f);
            cube.transform.Rotate(xAngle, yAngle, zAngle, Space.World);

            PosRot cubePos = new PosRot(
                cube.transform.position.x,
                cube.transform.position.y,
                cube.transform.position.z,
                cube.transform.rotation.x,
                cube.transform.rotation.y,
                cube.transform.rotation.z,
                cube.transform.rotation.w
            );

            // Finally send the message to server_endpoint.py running in ROS
            tcpCon.SendMessage(topicName, cubePos);

            timeElapsed = 0;
        }

    }
}