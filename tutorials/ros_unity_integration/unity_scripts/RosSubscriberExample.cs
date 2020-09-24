using System.Net.Sockets;
using System.Threading.Tasks;
using UnityEngine;
using RosColor = RosMessageTypes.RoboticsDemo.UnityColor;

public class RosSubscriberExample : RosSubscriber
{

    public GameObject cube;

    protected override async Task HandleConnectionAsync(TcpClient tcpClient)
    {
        await Task.Yield();

        using (var networkStream = tcpClient.GetStream())
        {
            RosColor colorMessage = (RosColor)ReadMessage(networkStream, new RosColor());
            Debug.Log("Color(" + colorMessage.r + ", "+ colorMessage.g + ", "+ colorMessage.b + ", "+ colorMessage.a +")");
            cube.GetComponent<Renderer>().material.color = new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a);
        }
    }

    void Start()
    {
        StartMessageServer(hostPort);
    }

}