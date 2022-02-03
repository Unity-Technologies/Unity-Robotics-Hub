using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector;
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
            m_Ros.listenForTFMessages = false;
        }

        [UnityTest]
        public IEnumerator RosIntegration_Publisher_Success()
        {
#if INTEGRATION_TEST
            m_Cube.AddComponent<RosPublisherExample>().cube = m_Cube;
            yield return new EnterPlayMode();
            while (Time.time < k_SimulationTime)
            {
                yield return null;
            }

            yield return new ExitPlayMode();
            LogAssert.NoUnexpectedReceived();
            Object.DestroyImmediate(Object.FindObjectOfType<RosPublisherExample>().gameObject);
#else
            ThrowNotImplementedException();
            yield return null;
#endif
        }

        [UnityTest]
        public IEnumerator RosIntegration_Subscriber_Success()
        {
#if INTEGRATION_TEST
            m_Cube.AddComponent<RosSubscriberExample>().cube = m_Cube;
            yield return new EnterPlayMode();

            // Avoid cross-validation from other ros services (e.g. ROS service server test)
            LogAssert.ignoreFailingMessages = true;

            var subscriber = Object.FindObjectOfType<RosSubscriberExample>();
            var color = subscriber.GetComponent<Renderer>().material.color;
            while (Time.time < k_SimulationTimeout && subscriber.GetComponent<Renderer>().material.color.Equals(color))
            {
                yield return null;
            }
            Assert.AreNotEqual(color, subscriber.GetComponent<Renderer>().material.color);
            LogAssert.ignoreFailingMessages = false;
            yield return new ExitPlayMode();
            Object.DestroyImmediate(Object.FindObjectOfType<RosSubscriberExample>().gameObject);
#else
            ThrowNotImplementedException();
            yield return null;
#endif
        }

        [UnityTest]
        public IEnumerator RosIntegration_ServiceServer_Success()
        {
#if INTEGRATION_TEST
            m_Cube.AddComponent<RosUnityServiceExample>();
            yield return new EnterPlayMode();
            while (Time.time < k_SimulationTimeout)
            {
                yield return null;
            }
            LogAssert.Expect(LogType.Log, new Regex(@"^Received request for object: .*$"));
            yield return new ExitPlayMode();
            Object.DestroyImmediate(Object.FindObjectOfType<RosUnityServiceExample>().gameObject);
#else
            ThrowNotImplementedException();
            yield return null;
#endif
        }

        [UnityTest]
        public IEnumerator RosIntegration_ServiceClient_Success()
        {
#if INTEGRATION_TEST
            m_Cube.AddComponent<RosServiceCallExample>().cube = m_Cube;
            yield return new EnterPlayMode();
            while (Time.time < k_SimulationTime)
            {
                yield return null;
            }
            LogAssert.Expect(LogType.Log, "Destination reached.");
            LogAssert.Expect(LogType.Log, new Regex(@"^New Destination: .*$"));
            yield return new ExitPlayMode();
            Object.DestroyImmediate(Object.FindObjectOfType<RosServiceCallExample>().gameObject);
#else
            ThrowNotImplementedException();
            yield return null;
#endif
        }

        [TearDown]
        public void TearDown()
        {
            Object.DestroyImmediate(m_Cube);
            Object.DestroyImmediate(m_Ros);
        }

        void ThrowNotImplementedException()
        {
            throw new NotImplementedException(
                "This integration test can only be executed with the INTEGRATION_TEST scripting define set. " +
                "The dependencies of this test are not guaranteed to exist in the Project by default.");
        }
    }
}
