using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

namespace IntegrationTests
{
    [TestFixture, Explicit, Category("IntegrationTests")]
    // IMPORTANT: In order for this category of tests to run correctly, MessageGeneration must be run first and the
    //            INTEGRATION_TEST script define must be set
    public class RosIntegrationTests
    {
        const float k_SimulationTime = 5f;
        const string k_RosProtocolVariableName = "ROS_PROTOCOL";

        [UnityTest]
        public IEnumerator RosIntegration_Publisher_Success()
        {
#if INTEGRATION_TEST
            var cube = new GameObject("Cube");
            var publisher = cube.AddComponent<RosPublisherExample>();
            publisher.cube = cube;
            var ros = ROSConnection.GetOrCreateInstance();
            UpdateRosProtocol();
            ros.listenForTFMessages = false;
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
