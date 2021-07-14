#if UNITY_EDITOR
using Microsoft.CSharp;
using System;
using System.CodeDom.Compiler;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;

public class Demo : MonoBehaviour
{
    public string hostIP = "127.0.0.1";

    public bool generateRosMessages = true;
    public bool deleteRosMessagesAfterSimulation = true;

    string prefabDirectory = "Assets/Prefabs";
    string prefabSuffix = ".prefab";

    string tableName = "Table";
    string targetName = "Target";
    string targetPlacementName = "TargetPlacement";

    string cameraName = "Main Camera";
    Vector3 cameraPosition = new Vector3(0, 1.4f, -0.7f);
    Quaternion cameraRotation = Quaternion.Euler(new Vector3(45, 0, 0));

    string urdfRelativeFilepath = "URDF/niryo_one/niryo_one.urdf";

    string niryoOneName = "niryo_one";
    string baseLinkName = "base_link";
    float controllerStiffness = 10000;
    float controllerDamping = 100;
    float controllerForceLimit = 1000;
    float controllerSpeed = 30;
    float controllerAcceleration = 10;

    string rosMessagesDirectory = "Assets/Scripts/RosMessages";
    string rosSrcDirectory = "../ROS/src";
    string msgDirectory = "msg";
    string srvDirectory = "srv";
    string moveitMsgPackageName = "moveit_msgs";
    string niryoMoveitPackageName = "niryo_moveit";
    string robotTrajectoryMessageFileName = "RobotTrajectory.msg";
    string moverServiceFileName = "MoverService.srv";
    string scriptPattern = "*.cs";

    string registerMethodName = "Register";
    string[] rosGeneratedTypeFullName = new string[]
    {
        "RosMessageTypes.Moveit.RobotTrajectoryMsg",
        "RosMessageTypes.NiryoMoveit.NiryoMoveitJointsMsg",
        "RosMessageTypes.NiryoMoveit.NiryoTrajectoryMsg",
        "RosMessageTypes.NiryoMoveit.MoverServiceResponse",
        "RosMessageTypes.NiryoMoveit.MoverServiceRequest"
    };


    string externalScriptsDirectory = "../Scripts";
    //string scriptsDirectory = "Assets/Scripts";

    string rosConnectName = "ROSConnect";
    string publisherName = "Publisher";
    int hostPort = 10000;

    string trajectoryPlannerType = "TrajectoryPlanner";
    string rosServiceName = "niryo_moveit";

    bool hasPublished = false;

    Assembly assembly;
    ROSConnection rosConnection;

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
        if (!hasPublished)
        {
            dynamic publisher = GameObject.Find(publisherName).GetComponent(assembly.GetType(trajectoryPlannerType));
            publisher.PublishJoints();
            hasPublished = true;
        }
    }

    void OnApplicationQuit()
    {
        if (deleteRosMessagesAfterSimulation)
        {
            Directory.Delete(rosMessagesDirectory, recursive: true);
            File.Delete($"{rosMessagesDirectory}.meta");
        }
        EditorApplication.UnlockReloadAssemblies();
    }

    // Tutorial Part 1
    private void SetupScene()
    {
        // Instantiate the table, target, and target placement
        InstantiatePrefab(tableName);
        InstantiatePrefab(targetName);
        InstantiatePrefab(targetPlacementName);

        // Adjust main camera
        GameObject camera = GameObject.Find(cameraName);
        camera.transform.position = cameraPosition;
        camera.transform.rotation = cameraRotation;
    }

    // Tutorial Part 2 without the Publisher object
    private void AddRosMessages()
    {
        if (generateRosMessages)
        {
            // Generate ROS messages
            MessageAutoGen.GenerateSingleMessage(
                Path.Combine(rosSrcDirectory, moveitMsgPackageName, msgDirectory, robotTrajectoryMessageFileName),
                rosMessagesDirectory, moveitMsgPackageName);

            MessageAutoGen.GenerateDirectoryMessages(
                Path.Combine(rosSrcDirectory, niryoMoveitPackageName, msgDirectory),
                rosMessagesDirectory);

            // Generate ROS services
            ServiceAutoGen.GenerateSingleService(
                Path.Combine(rosSrcDirectory, niryoMoveitPackageName, srvDirectory, moverServiceFileName),
                rosMessagesDirectory, niryoMoveitPackageName);
        }

        // Recompile ROS message scripts and external scripts
        List<string> scripts = new List<string>();
        scripts.AddRange(Directory.GetFiles(rosMessagesDirectory, scriptPattern, SearchOption.AllDirectories));
        scripts.AddRange(Directory.GetFiles(externalScriptsDirectory));
        RecompileScripts(scripts.ToArray());

        // Register Ros message names and their deserialize function
        foreach (var typeFullName in rosGeneratedTypeFullName)
        {
            assembly.GetType(typeFullName).GetMethod(registerMethodName).Invoke(null, null);
        }
    }

    private void CreateTrajectoryPlannerPublisher()
    {
        GameObject publisher = new GameObject(publisherName);
        dynamic planner = publisher.AddComponent(assembly.GetType(trajectoryPlannerType));
        planner.rosServiceName = rosServiceName;
        planner.niryoOne = GameObject.Find(niryoOneName);
        planner.target = GameObject.Find(targetName);
        planner.targetPlacement = GameObject.Find(targetPlacementName);
    }

    private GameObject InstantiatePrefab(string name)
    {
        string filepath = Path.Combine(prefabDirectory, $"{name}{prefabSuffix}");
        GameObject gameObject = Instantiate(AssetDatabase.LoadAssetAtPath<GameObject>(filepath));
        gameObject.name = name;
        return gameObject;
    }

    private void CreateRosConnection()
    {
        // Create RosConnect
        GameObject rosConnect = new GameObject(rosConnectName);
        rosConnection = rosConnect.AddComponent<ROSConnection>();
        rosConnection.RosIPAddress = hostIP;
        rosConnection.RosPort = hostPort;
    }

    private void ImportRobot()
    {
        ImportSettings urdfImportSettings = new ImportSettings
        {
            choosenAxis = ImportSettings.axisType.yAxis,
            convexMethod = ImportSettings.convexDecomposer.unity
        };
        // Import Niryo One with URDF importer
        string urdfFilepath = Path.Combine(Application.dataPath, urdfRelativeFilepath);
        // Create is a coroutine that would usually run only in EditMode, so we need to force its execution here
        var robotImporter = UrdfRobotExtensions.Create(urdfFilepath, urdfImportSettings, false);
        while (robotImporter.MoveNext()) { }
        // Adjust robot parameters
        Controller controller = GameObject.Find(niryoOneName).GetComponent<Controller>();
        controller.stiffness = controllerStiffness;
        controller.damping = controllerDamping;
        controller.forceLimit = controllerForceLimit;
        controller.speed = controllerSpeed;
        controller.acceleration = controllerAcceleration;
        GameObject.Find(baseLinkName).GetComponent<ArticulationBody>().immovable = true;
    }

    // Credit to https://www.codeproject.com/Tips/715891/Compiling-Csharp-Code-at-Runtime
    // and https://gamedev.stackexchange.com/questions/130268/how-do-i-compile-a-c-script-at-runtime-and-attach-it-as-a-component-to-a-game-o
    private void RecompileScripts(string[] filepaths)
    {
        CSharpCodeProvider provider = new CSharpCodeProvider();
        CompilerParameters parameters = new CompilerParameters();
        foreach (Assembly unityAssembly in AppDomain.CurrentDomain.GetAssemblies())
        {
            parameters.ReferencedAssemblies.Add(unityAssembly.Location);
        }
        parameters.GenerateInMemory = true;
        parameters.GenerateExecutable = false;

        string[] texts = new string[filepaths.Length];
        for (int i = 0; i < filepaths.Length; i++)
        {
            texts[i] = File.ReadAllText(filepaths[i]);
        }
        CompilerResults results = provider.CompileAssemblyFromSource(parameters, texts);
        checkCompileErrors(results);

        // Update the assembly
        assembly = results.CompiledAssembly;
    }

    private void checkCompileErrors(CompilerResults results)
    {
        if (results.Errors.HasErrors)
        {
            StringBuilder stringBuilder = new StringBuilder();
            foreach (CompilerError error in results.Errors)
            {
                stringBuilder.AppendLine($"Error {error.ErrorNumber} {error.ErrorText}");
            }
            throw new InvalidOperationException(stringBuilder.ToString());
        }
    }
}
#endif
