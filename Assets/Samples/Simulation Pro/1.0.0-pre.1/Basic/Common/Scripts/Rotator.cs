using UnityEngine;

public class Rotator : MonoBehaviour
{
    [Tooltip("Rotation speed: angles (degree) per second")]
    [SerializeField]
    float m_Speed = 100f;

    [Tooltip("Rotation Axis")]
    [SerializeField]
    Vector3 m_Axis = Vector3.one;

    void Update()
    {
        transform.Rotate(Time.deltaTime * m_Speed * m_Axis.x,
            Time.deltaTime * m_Speed * m_Axis.y,
            Time.deltaTime * m_Speed * m_Axis.z);
        if (gameObject.TryGetComponent(out ArticulationBody ab))
        {
            ab.TeleportRoot(transform.position, transform.rotation);
        }
    }
}
