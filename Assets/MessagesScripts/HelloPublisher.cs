using UnityEngine;
using Unity.SimulationPro.Foundation;
using Unity.SimulationPro.Foundation.StdMessages;

public class HelloPublisher : PublisherBehaviour<StringMsg>
{

    public override string DefaultTopic => "/hello";

    // Update is called once per frame
    void Update()
    {
        if (CanPublish)
        {
            Publish(new StringMsg("Hello from Unity!"));
        }
    }
}
