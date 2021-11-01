using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

namespace IntegrationTests
{
    [TestFixture, Explicit, Category("IntegrationTests")]
    // IMPORTANT: In order for this category of tests to run correctly, MessageGeneration must be run first and the
    //            INTEGRATION_TEST script define must be set
    public class RosIntegrationTests
    {
        GameObject m_Cube;
        ROSConnection m_Ros;

        const float k_SimulationTime = 5f;
        const float k_SimulationTimeout = 60f;
        const string k_RosProtocolVariableName = "ROS_PROTOCOL";

        [SetUp]
        public void SetUp()
        {
            m_Cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            m_Ros = ROSConnection.GetOrCreateInstance();
            UpdateRosProtocol();
            m_Ros.listenForTFMessages = false;
        }

        [UnityTest]
        public IEnumerator RosIntegration_Publisher_Success()
        {
#if INTEGRATION_TEST
            var publisher = m_Cube.AddComponent<RosPublisherExample>();
            publisher.cube = m_Cube;
            yield return new EnterPlayMode();
            while (Time.time < k_SimulationTime)
            {
                yield return null;
            }

            yield return new ExitPlayMode();
            LogAssert.NoUnexpectedReceived();
#else
            throw new NotImplementedException(
                "This integration test can only be executed with the INTEGRATION_TEST scripting define set. " +
                "The dependencies of this test are not guaranteed to exist in the Project by default.");
#endif
        }

        [UnityTest]
        public IEnumerator RosIntegration_Subscriber_Success()
        {
#if INTEGRATION_TEST
            m_Cube.AddComponent<RosSubscriberExample>().cube = m_Cube;
            yield return new EnterPlayMode();
            var subscriber = Object.FindObjectOfType<RosSubscriberExample>();
            var color = subscriber.GetComponent<Renderer>().material.color;
            while (Time.time < k_SimulationTimeout && subscriber.GetComponent<Renderer>().material.color.Equals(color))
            {
                yield return null;
            }
            Assert.AreNotEqual(color, subscriber.GetComponent<Renderer>().material.color);
            yield return new ExitPlayMode();
#else
            throw new NotImplementedException(
                "This integration test can only be executed with the INTEGRATION_TEST scripting define set. " +
                "The dependencies of this test are not guaranteed to exist in the Project by default.");
#endif
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(m_Cube);
            Object.DestroyImmediate(m_Ros);
        }

        void UpdateRosProtocol()
        {
            string protocol = Environment.GetEnvironmentVariable(k_RosProtocolVariableName);
            var buildTarget = EditorUserBuildSettings.activeBuildTarget;
            var buildTargetGroup = BuildPipeline.GetBuildTargetGroup(buildTarget);
            List<string> allDefines = PlayerSettings.GetScriptingDefineSymbolsForGroup(buildTargetGroup).Split(';').ToList();
            if (protocol == "ROS2")
            {
                allDefines.Add("ROS2");
            }
            else
            {
                allDefines.Remove("ROS2");
            }
            PlayerSettings.SetScriptingDefineSymbolsForGroup(buildTargetGroup, string.Join(";", allDefines));
        }
    }
}
