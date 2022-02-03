#if UNITY_EDITOR
using System;
using System.CodeDom.Compiler;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using Microsoft.CSharp;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;
using UnityEditor;
using UnityEngine;

public class Demo : MonoBehaviour
{
    const string k_BaseLinkName = "base_link";

    const string k_CameraName = "Main Camera";
    const float k_ControllerAcceleration = 10;
    const float k_ControllerDamping = 100;
    const float k_ControllerForceLimit = 1000;
    const float k_ControllerSpeed = 30;
    const float k_ControllerStiffness = 10000;

    const string k_ExternalScriptsDirectory = "../Scripts";
    const int k_HostPort = 10000;
    const string k_MoveitMsgPackageName = "moveit_msgs";
    const string k_MoverServiceFileName = "MoverService.srv";
    const string k_MsgDirectory = "msg";
    const string k_NiryoMoveitPackageName = "niryo_moveit";

    const string k_NiryoOneName = "niryo_one";

    const string k_PrefabDirectory = "Assets/Prefabs";
    const string k_PrefabSuffix = ".prefab";
    const string k_PublisherName = "Publisher";

    const string k_RegisterMethodName = "Register";
    const string k_RobotTrajectoryMessageFileName = "RobotTrajectory.msg";

    //string scriptsDirectory = "Assets/Scripts";

    const string k_RosConnectName = "ROSConnect";

    const string k_RosMessagesDirectory = "Assets/Scripts/RosMessages";
    const string k_RosServiceName = "niryo_moveit";
    const string k_RosSrcDirectory = "../ROS/src";
    const string k_ScriptPattern = "*.cs";
    const string k_SrvDirectory = "srv";

    const string k_TableName = "Table";
    const string k_TargetName = "Target";
    const string k_TargetPlacementName = "TargetPlacement";

    const string k_TrajectoryPlannerType = "TrajectoryPlanner";

    const string k_UrdfRelativeFilepath = "URDF/niryo_one/niryo_one.urdf";
    [SerializeField]
    string m_HostIP = "127.0.0.1";

    [SerializeField]
    bool m_GenerateRosMessages = true;
    [SerializeField]
    bool m_DeleteRosMessagesAfterSimulation = true;
    readonly string[] m_RosGeneratedTypeFullName =
    {
        "RosMessageTypes.Moveit.RobotTrajectoryMsg",
        "RosMessageTypes.NiryoMoveit.NiryoMoveitJointsMsg",
        "RosMessageTypes.NiryoMoveit.NiryoTrajectoryMsg",
        "RosMessageTypes.NiryoMoveit.MoverServiceResponse",
        "RosMessageTypes.NiryoMoveit.MoverServiceRequest"
    };

    Assembly m_Assembly;
    Vector3 m_CameraPosition = new Vector3(0, 1.4f, -0.7f);
    Quaternion m_CameraRotation = Quaternion.Euler(new Vector3(45, 0, 0));

    bool m_HasPublished;
    ROSConnection m_RosConnection;

    void Awake()
    {
        EditorApplication.LockReloadAssemblies();
        SetupScene();
        AddRosMessages();
        ImportRobot();
        CreateRosConnection();
        CreateTrajectoryPlannerPublisher();
    }

    void Update()
    {
        // Make sure to publish only once in the demo
        if (!m_HasPublished)
        {
            dynamic publisher = GameObject.Find(k_PublisherName)
                .GetComponent(m_Assembly.GetType(k_TrajectoryPlannerType));
            publisher.PublishJoints();
            m_HasPublished = true;
        }
    }

    void OnApplicationQuit()
    {
        if (m_DeleteRosMessagesAfterSimulation)
        {
            Directory.Delete(k_RosMessagesDirectory, true);
            File.Delete($"{k_RosMessagesDirectory}.meta");
        }

        EditorApplication.UnlockReloadAssemblies();
    }

    // Tutorial Part 1
    void SetupScene()
    {
        // Instantiate the table, target, and target placement
        InstantiatePrefab(k_TableName);
        InstantiatePrefab(k_TargetName);
        InstantiatePrefab(k_TargetPlacementName);

        // Adjust main camera
        var camera = GameObject.Find(k_CameraName);
        camera.transform.position = m_CameraPosition;
        camera.transform.rotation = m_CameraRotation;
    }

    // Tutorial Part 2 without the Publisher object
    void AddRosMessages()
    {
        if (m_GenerateRosMessages)
        {
            // Generate ROS messages
            MessageAutoGen.GenerateSingleMessage(
                Path.Combine(k_RosSrcDirectory, k_MoveitMsgPackageName, k_MsgDirectory,
                    k_RobotTrajectoryMessageFileName),
                k_RosMessagesDirectory, k_MoveitMsgPackageName);

            MessageAutoGen.GenerateDirectoryMessages(
                Path.Combine(k_RosSrcDirectory, k_NiryoMoveitPackageName, k_MsgDirectory),
                k_RosMessagesDirectory);

            // Generate ROS services
            ServiceAutoGen.GenerateSingleService(
                Path.Combine(k_RosSrcDirectory, k_NiryoMoveitPackageName, k_SrvDirectory, k_MoverServiceFileName),
                k_RosMessagesDirectory, k_NiryoMoveitPackageName);
        }

        // Recompile ROS message scripts and external scripts
        var scripts = new List<string>();
        scripts.AddRange(Directory.GetFiles(k_RosMessagesDirectory, k_ScriptPattern, SearchOption.AllDirectories));
        scripts.AddRange(Directory.GetFiles(k_ExternalScriptsDirectory));
        RecompileScripts(scripts.ToArray());

        // Register Ros message names and their deserialize function
        foreach (var typeFullName in m_RosGeneratedTypeFullName)
        {
            m_Assembly.GetType(typeFullName).GetMethod(k_RegisterMethodName).Invoke(null, null);
        }
    }

    void CreateTrajectoryPlannerPublisher()
    {
        var publisher = new GameObject(k_PublisherName);
        dynamic planner = publisher.AddComponent(m_Assembly.GetType(k_TrajectoryPlannerType));
        planner.RosServiceName = k_RosServiceName;
        planner.NiryoOne = GameObject.Find(k_NiryoOneName);
        planner.Target = GameObject.Find(k_TargetName);
        planner.TargetPlacement = GameObject.Find(k_TargetPlacementName);
    }

    GameObject InstantiatePrefab(string name)
    {
        var filepath = Path.Combine(k_PrefabDirectory, $"{name}{k_PrefabSuffix}");
        var gameObject = Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(filepath));
        gameObject.name = name;
        return gameObject;
    }

    void CreateRosConnection()
    {
        // Create RosConnect
        var rosConnect = new GameObject(k_RosConnectName);
        m_RosConnection = rosConnect.AddComponent<ROSConnection>();
        m_RosConnection.RosIPAddress = m_HostIP;
        m_RosConnection.RosPort = k_HostPort;
    }

    void ImportRobot()
    {
        var urdfImportSettings = new ImportSettings
        {
            chosenAxis = ImportSettings.axisType.yAxis,
            convexMethod = ImportSettings.convexDecomposer.unity
        };

        // Import Niryo One with URDF importer
        var urdfFilepath = Path.Combine(Application.dataPath, k_UrdfRelativeFilepath);

        // Create is a coroutine that would usually run only in EditMode, so we need to force its execution here
        var robotImporter = UrdfRobotExtensions.Create(urdfFilepath, urdfImportSettings);
        while (robotImporter.MoveNext()) { }

        // Adjust robot parameters
        var controller = GameObject.Find(k_NiryoOneName).GetComponent<Controller>();
        controller.stiffness = k_ControllerStiffness;
        controller.damping = k_ControllerDamping;
        controller.forceLimit = k_ControllerForceLimit;
        controller.speed = k_ControllerSpeed;
        controller.acceleration = k_ControllerAcceleration;
        GameObject.Find(k_BaseLinkName).GetComponent<ArticulationBody>().immovable = true;
    }

    // Credit to https://www.codeproject.com/Tips/715891/Compiling-Csharp-Code-at-Runtime
    // and https://gamedev.stackexchange.com/questions/130268/how-do-i-compile-a-c-script-at-runtime-and-attach-it-as-a-component-to-a-game-o
    void RecompileScripts(string[] filepaths)
    {
        var provider = new CSharpCodeProvider();
        var parameters = new CompilerParameters();
        foreach (var unityAssembly in AppDomain.CurrentDomain.GetAssemblies())
        {
            parameters.ReferencedAssemblies.Add(unityAssembly.Location);
        }
        parameters.GenerateInMemory = true;
        parameters.GenerateExecutable = false;

        var texts = new string[filepaths.Length];
        for (var i = 0; i < filepaths.Length; i++)
        {
            texts[i] = File.ReadAllText(filepaths[i]);
        }
        var results = provider.CompileAssemblyFromSource(parameters, texts);
        checkCompileErrors(results);

        // Update the assembly
        m_Assembly = results.CompiledAssembly;
    }

    void checkCompileErrors(CompilerResults results)
    {
        if (results.Errors.HasErrors)
        {
            var stringBuilder = new StringBuilder();
            foreach (CompilerError error in results.Errors)
            {
                stringBuilder.AppendLine($"Error {error.ErrorNumber} {error.ErrorText}");
            }
            throw new InvalidOperationException(stringBuilder.ToString());
        }
    }
}
#endif
