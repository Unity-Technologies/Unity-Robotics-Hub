using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.Editor.MessageGeneration;
using UnityEditor;
using UnityEngine;

namespace MessageGenerationTests
{
    // This gets a special category to enable running independently when needed to generate message definitions
    [TestFixture, Category("MessageGeneration")]
    public class MessageGenerationTests
    {
        enum PathType
        {
            File,
            Directory
        }

        // Relative path to the directory containing the catkin packages
        static readonly string k_PickAndPlaceROSDirectory = Path.GetFullPath(Path.Combine("..", "ROS", "src"));
        static readonly string k_ROSIntegrationDirectory = Path.GetFullPath(Path.Combine("..", "..", "ros_unity_integration", "ros_packages"));
        static string m_MessageGenOutputPath => MessageGenBrowserSettings.Get().outputPath;

        static void WarnIfAlreadyExists(string path, PathType pathType)
        {
            var alreadyExists = pathType == PathType.File ? File.Exists(path) : Directory.Exists(path);
            if (alreadyExists)
            {
                Debug.LogWarning($"{path} already exists. Test can't validate files were generated correctly unless " +
                    "this path is manually deleted first.");
            }
        }

        static void AssertExists(string path, PathType pathType)
        {
            Assert.IsTrue(pathType == PathType.File ? File.Exists(path) : Directory.Exists(path));
        }

        // Define more individual messages to run generation on within this test case enumerable
        static IEnumerable<TestCaseData> IndividualMessages()
        {
            yield return new TestCaseData(Path.Combine(k_PickAndPlaceROSDirectory, "moveit_msgs", "msg", "RobotTrajectory.msg"));
        }

        // Define directories of message files to be generated here
        static IEnumerable<TestCaseData> MessageDirectories()
        {
            yield return new TestCaseData(Path.Combine(k_PickAndPlaceROSDirectory, "niryo_moveit", "msg"));
            yield return new TestCaseData(Path.Combine(k_ROSIntegrationDirectory, "unity_robotics_demo_msgs", "msg"));
        }

        // Define directories of service files to be generated here
        static IEnumerable<TestCaseData> ServiceDirectories()
        {
            yield return new TestCaseData(Path.Combine(k_PickAndPlaceROSDirectory, "niryo_moveit", "srv"));
            yield return new TestCaseData(Path.Combine(k_ROSIntegrationDirectory, "unity_robotics_demo_msgs", "srv"));
        }

        [Test]
        [TestCaseSource(nameof(IndividualMessages))]
        public void TestMessageBuildSingle_ThrowsNoExceptions(string messageToBuild)
        {
            var msgPath = MessageAutoGen.GetMessageClassPath(messageToBuild, m_MessageGenOutputPath);
            Debug.Log($"Generating code for {messageToBuild}, output should be at {msgPath}");
            WarnIfAlreadyExists(msgPath, PathType.File);
            MessageAutoGen.GenerateSingleMessage(messageToBuild, m_MessageGenOutputPath);
            AssetDatabase.Refresh();
            AssertExists(msgPath, PathType.File);
        }

        [Test]
        [TestCaseSource(nameof(MessageDirectories))]
        public void TestMessageBuildDirectory_ThrowsNoExceptions(string directoryToBuild)
        {
            var msgPath = MessageAutoGen.GetMessageClassPath(directoryToBuild, m_MessageGenOutputPath);
            var msgDirectory = Path.GetDirectoryName(msgPath);
            Debug.Log($"Generating code in {directoryToBuild}, output should be in {msgDirectory}");
            WarnIfAlreadyExists(msgDirectory, PathType.Directory);
            MessageAutoGen.GenerateDirectoryMessages(directoryToBuild, m_MessageGenOutputPath);
            AssetDatabase.Refresh();
            AssertExists(msgDirectory, PathType.Directory);
        }

        [Test]
        [TestCaseSource(nameof(ServiceDirectories))]
        public void TestServiceBuildDirectory_ThrowsNoExceptions(string directoryToBuild)
        {
            var msgPath = ServiceAutoGen.GetServiceClassPaths(directoryToBuild, m_MessageGenOutputPath);
            var msgDirectory = Path.GetDirectoryName(msgPath[0]);
            Debug.Log($"Generating code in {directoryToBuild}, output should be in {msgDirectory}");
            WarnIfAlreadyExists(msgDirectory, PathType.Directory);
            ServiceAutoGen.GenerateDirectoryServices(directoryToBuild, m_MessageGenOutputPath);
            AssetDatabase.Refresh();
            AssertExists(msgDirectory, PathType.Directory);
        }
    }
}
