using UnityEngine;
using Unity.SimulationPro.Foundation;
using Unity.SimulationPro.Foundation.StdMessages;
using System;

public class HelloSubscriber : SubscriberBehaviour<StringMsg>
{
    public override string DefaultTopic => "/hello";

    protected override Action<StringMsg> Callback => 
        msg => Debug.Log($"Received message: {msg.data}");

}
