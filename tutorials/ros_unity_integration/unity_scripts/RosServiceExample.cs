using System;
using RosMessageTypes.RoboticsDemo;
using UnityEngine;
using Debug = UnityEngine.Debug;

public class RosServiceExample : MonoBehaviour
{
    private TcpConnector tcpCon;

    public string serviceName = "pos_srv";
    public string hostName = "192.168.1.116";
    public int hostPort = 10000;

    public GameObject cube;

    // Cube movement conditions
    public float delta = 1.0f;
    public float speed = 2.0f;
    private Vector3 destination;

    void Start()
    {
        tcpCon = new TcpConnector(hostName, hostPort);

        destination = cube.transform.position;
    }

    private void FixedUpdate()
    {

        // Only compare x and z since our cube should not deviate on the y axis
        if (Math.Abs(cube.transform.position.x - destination.x) < delta &&
            Math.Abs(cube.transform.position.z - destination.z) < delta)
        {
            Debug.Log("Destination reached.");

            PosRot cubePos = new PosRot(
                cube.transform.position.x,
                cube.transform.position.y,
                cube.transform.position.z,
                cube.transform.rotation.x,
                cube.transform.rotation.y,
                cube.transform.rotation.z,
                cube.transform.rotation.w
            );

            PositionServiceRequest positionServiceRequest = new PositionServiceRequest(cubePos);

            // Send message to ROS and return the response
            var response = (PositionServiceResponse) tcpCon.SendServiceMessage(serviceName, positionServiceRequest, new PositionServiceResponse());

            destination = new Vector3(response.output.pos_x, response.output.pos_y, response.output.pos_z);

            Debug.Log("New Destination: " + destination);
        }
        // Translate cube to destination
        else
        {
            // Move our position a step closer to the target.
            float step =  speed * Time.deltaTime; // calculate distance to move
            cube.transform.position = Vector3.MoveTowards(cube.transform.position, destination, step);
        }
    }
}