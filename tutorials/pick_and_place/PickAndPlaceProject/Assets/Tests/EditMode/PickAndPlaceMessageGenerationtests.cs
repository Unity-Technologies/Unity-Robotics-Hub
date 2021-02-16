using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.Editor.MessageGeneration;
using UnityEditor;

[TestFixture]
[Category("MessageGeneration")]
public class PickAndPlaceMessageGenerationTests
{
    // Relative path to the directory containing the catkin packages
    static readonly string k_ROSDirectory = Path.GetFullPath(Path.Combine("..", "ROS", "src"));
    static string m_MessageGenOutputPath => MessageGenBrowserSettings.Get().outputPath;

    // Define more individual messages to run generation on within this test case enumerable
    static IEnumerable<TestCaseData> IndividualMessages()
    {
        yield return new TestCaseData(Path.Combine(k_ROSDirectory, "moveit_msgs", "msg", "RobotTrajectory.msg"));
    }
    
    // Define directories of message files to be generated here
    static IEnumerable<TestCaseData> MessageDirectories()
    {
        yield return new TestCaseData(Path.Combine(k_ROSDirectory, "niryo_moveit", "msg"));
    }

    // Define directories of service files to be generated here
    static IEnumerable<TestCaseData> ServiceDirectories()
    {
        yield return new TestCaseData(Path.Combine(k_ROSDirectory, "niryo_moveit", "srv"));
    }

    [Test]
    [TestCaseSource(nameof(IndividualMessages))]
    public void TestMessageBuildSingle_ThrowsNoExceptions(string messageToBuild)
    {
        MessageAutoGen.GenerateSingleMessage(messageToBuild, m_MessageGenOutputPath);
        AssetDatabase.Refresh();
    }

    [Test]
    [TestCaseSource(nameof(MessageDirectories))]
    public void TestMessageBuildDirectory_ThrowsNoExceptions(string directoryToBuild)
    {
        MessageAutoGen.GenerateDirectoryMessages(directoryToBuild, m_MessageGenOutputPath);
        AssetDatabase.Refresh();
    }
    
    [Test]
    [TestCaseSource(nameof(ServiceDirectories))]
    public void TestServiceBuildDirectory_ThrowsNoExceptions(string directoryToBuild)
    {
        ServiceAutoGen.GenerateDirectoryServices(directoryToBuild, m_MessageGenOutputPath);
        AssetDatabase.Refresh();
    }
}
