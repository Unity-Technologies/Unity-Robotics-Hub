using System;
using System.Collections;
using System.IO;
using NUnit.Framework;
using Unity.Robotics.PickAndPlace;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.TestTools;

namespace IntegrationTests
{
    [TestFixture, Explicit, Category("IntegrationTests")]
    // IMPORTANT: In order for this category of tests to run correctly, MessageGeneration must be run first and the
    //            INTEGRATION_TEST script define must be set
    public class PickAndPlaceIntegrationTests
    {
        #region Parameters

        // Testing parameters
        const float k_TestTimeoutSeconds = 20f;

        const string k_NamePackageNiryoMoveIt = "niryo_moveit";
        const string k_NameNiryoOne = "niryo_one";
        const string k_NameBaseLink = "base_link";

        // Prefabs that get instantiated into scene
        const string k_NameTable = "Table";
        const string k_NameTarget = "Target";
        const string k_NameTargetPlacement = "TargetPlacement";

        // GameObjects that hold important components
        const string k_NameCamera = "Main Camera";
        const string k_NameRosConnect = "ROSConnect";
        const string k_NamePublisher = "Publisher";

        // Parameters for robot joint controller
        const float k_ControllerStiffness = 10000f;
        const float k_ControllerDamping = 100f;
        const float k_ControllerForceLimit = 1000f;
        const float k_ControllerSpeed = 30f;
        const float k_ControllerAcceleration = 10f;

        // Parameters for ROS connection
        const string k_IpAddressLoopback = "127.0.0.1";
        const int k_HostPort = 10000;
        const int k_UnityPort = 5005;
        const int k_NumAwaitDataRetries = 10;
        const int k_NumAwaitDataSleepSeconds = 1;

        const string k_PrefabSuffix = ".prefab";
        readonly string k_DirectoryPrefabs = Path.Combine("Assets", "Prefabs");
        readonly string k_PathUrdf = Path.Combine("URDF", "niryo_one", "niryo_one.urdf");
        readonly string k_PathTestScene = Path.Combine("Assets", "Scenes", "EmptyScene.unity");

        readonly Vector3 k_CameraPosition = new Vector3(0, 1.4f, -0.7f);
        readonly Quaternion k_CameraRotation = Quaternion.Euler(new Vector3(45, 0, 0));

        readonly ImportSettings k_UrdfImportSettings = new ImportSettings
        {
            chosenAxis = ImportSettings.axisType.yAxis,
            convexMethod = ImportSettings.convexDecomposer.unity
        };
        #endregion

        float m_TimeElapsedSeconds;
        [SerializeField, HideInInspector]
        ROSConnection m_RosConnection;
        [SerializeField, HideInInspector]
        TargetPlacement m_TargetPlacement;

        // NOTE: This check could be made more robust by checking the gripper state of the arm to confirm it has
        //       released the Target
        bool DidPlacementSucceed => m_TargetPlacement.CurrentState == TargetPlacement.PlacementState.InsidePlaced;


        [UnityTest]
        public IEnumerator TrajectoryPublisher_PickAndPlaceDemo_CompletesTask()
        {
#if INTEGRATION_TEST
            SetUpScene();
            // TODO: This test could be made a PlayMode test once ImportRobot can use the PlayMode URDF import
            ImportRobot();
            CreateRosConnection();
            CreateTrajectoryPlannerPublisher();
            yield return new EnterPlayMode();

            m_TargetPlacement = GameObject.Find(k_NameTargetPlacement).GetComponent<TargetPlacement>();
            Assert.IsNotNull(m_TargetPlacement, $"Unable to find {nameof(TargetPlacement)} attached to a " +
                $"GameObject called {k_NameTargetPlacement} in scene.");

            var publisher = GameObject.Find(k_NamePublisher).GetComponent<TrajectoryPlanner>();
            publisher.PublishJoints();

            while(!DidPlacementSucceed && m_TimeElapsedSeconds < k_TestTimeoutSeconds)
            {
                m_TimeElapsedSeconds += Time.deltaTime;
                yield return null;
            }

            Assert.IsTrue(DidPlacementSucceed, "Pick and Place did not complete before test timed out.");

            yield return new ExitPlayMode();
        }

        void SetUpScene()
        {
            EditorSceneManager.OpenScene(k_PathTestScene);

            InstantiatePrefabFromName(k_NameTable);
            InstantiatePrefabFromName(k_NameTarget);
            InstantiatePrefabFromName(k_NameTargetPlacement);

            var camera = GameObject.Find(k_NameCamera);
            camera.transform.position = k_CameraPosition;
            camera.transform.rotation = k_CameraRotation;
        }

        void CreateTrajectoryPlannerPublisher()
        {
            var planner = new GameObject(k_NamePublisher).AddComponent<TrajectoryPlanner>();
            planner.RosServiceName = k_NamePackageNiryoMoveIt;
            planner.NiryoOne = GameObject.Find(k_NameNiryoOne);
            planner.Target = GameObject.Find(k_NameTarget);
            planner.TargetPlacement = GameObject.Find(k_NameTargetPlacement);
        }

        GameObject InstantiatePrefabFromName(string name)
        {
            var filepath = Path.Combine(k_DirectoryPrefabs, $"{name}{k_PrefabSuffix}");
            var gameObject = (GameObject) PrefabUtility.InstantiatePrefab(
                AssetDatabase.LoadAssetAtPath<GameObject>(filepath));
            gameObject.name = name;
            return gameObject;
        }

        void CreateRosConnection()
        {
            m_RosConnection = new GameObject(k_NameRosConnect).AddComponent<ROSConnection>();
            m_RosConnection.RosIPAddress = k_IpAddressLoopback;
            m_RosConnection.RosPort = k_HostPort;
            //m_RosConnection.overrideUnityIP = k_IpAddressLoopback;
            //m_RosConnection.unityPort = k_UnityPort;
            //m_RosConnection.awaitDataMaxRetries = k_NumAwaitDataRetries;
            //m_RosConnection.awaitDataSleepSeconds = k_NumAwaitDataSleepSeconds;
        }

        void ImportRobot()
        {
            var urdfFullPath = Path.Combine(Application.dataPath, k_PathUrdf);
            var robotImporter = UrdfRobotExtensions.Create(urdfFullPath, k_UrdfImportSettings, false);
            // Create is a coroutine that would usually run only in EditMode, so we need to force its execution here
            while (robotImporter.MoveNext())
            {
            }

            // Ensure parameters are set to reasonable values
            var controller = GameObject.Find(k_NameNiryoOne).GetComponent<Controller>();
            controller.stiffness = k_ControllerStiffness;
            controller.damping = k_ControllerDamping;
            controller.forceLimit = k_ControllerForceLimit;
            controller.speed = k_ControllerSpeed;
            controller.acceleration = k_ControllerAcceleration;
            GameObject.Find(k_NameBaseLink).GetComponent<ArticulationBody>().immovable = true;
#else
            throw new NotImplementedException(
                "This integration test can only be executed with the INTEGRATION_TEST scripting define set. " +
                "The dependencies of this test are not guaranteed to exist in the Project by default.");
#endif
        }
    }
}
