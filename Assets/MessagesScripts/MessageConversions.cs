using UnityEngine;
using Unity.SimulationPro.Foundation;
using Unity.SimulationPro.Foundation.GeometryMessages;

public class MessageConversions : MonoBehaviour
{
    private void Start() {
        var ourMsg = ToMessage(transform.position);
        ToUnity(ourMsg);
    }
    PointMsg ToMessage(Vector3 position)
    {
        Debug.Log(position.To<FLU>());
        return position.To<FLU>();
    }

    Vector3 ToUnity(PointMsg msg)
    {
        Debug.Log(msg.From<FLU>());
        return msg.From<FLU>();
    }
}
