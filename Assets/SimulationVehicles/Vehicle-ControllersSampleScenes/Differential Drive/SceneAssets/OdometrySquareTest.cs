using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Simulation.VehicleControllers;
using UnityEngine;

public class OdometrySquareTest : MonoBehaviour
{
    private DifferentialDriveController m_Controller;

    private struct Command
    {
        public Vector3 linearSpeed;
        public Vector3 angularSpeed;
        public float executionTime;
    }

    private List<Command> m_Queue = new List<Command>();

    void Start()
    {
        m_Controller = GetComponent<DifferentialDriveController>();
        for (int i = 0; i < 4; i++)
        {
            m_Queue.Add(new Command()
            {
                linearSpeed = Vector3.forward,
                angularSpeed = Vector3.zero,
                executionTime = 4
            });
            m_Queue.Add(new Command()
            {
                linearSpeed = Vector3.zero,
                angularSpeed = Vector3.up * 0.3925f,
                executionTime = 4
            });
        }

        StartCoroutine(LaunchCommands());
    }

    IEnumerator LaunchCommands()
    {
        yield return null;
        while (m_Queue.Count > 0)
        {
            var command = m_Queue.First();
            DifferentialDriveControlMessage msg = new DifferentialDriveControlMessage()
            {
                linear = command.linearSpeed,
                angular = command.angularSpeed
            };
            m_Controller.ConsumeMessage(msg);
            yield return new WaitForSeconds(command.executionTime);
            m_Queue.RemoveAt(0);
        }

        DifferentialDriveControlMessage msgZero = new DifferentialDriveControlMessage()
        {
            linear = Vector3.zero,
            angular = Vector3.zero
        };
        m_Controller.ConsumeMessage(msgZero);
        yield return null;
    }
}
