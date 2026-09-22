using UnityEditor;
using UnityEngine;

public class EditorUpdater
{
    static readonly int k_OusterIndex = 0;
    static readonly int k_VelodyneIndex = 1;
    static readonly int k_YDLidarIndex = 2;

    static readonly string[] k_SensorPrefabs = {
        "Ouster OS0 128",
        "Velodyne Puck",
        "YDLIDAR X4"
    };

    [MenuItem("GameObject/Simulation/Prebuilt LiDAR Sensors/Ouster OS0 128")]
    static void CreateOusterMenuItem()
    {
        var guid = CheckIfPrefabExists(k_OusterIndex);
        if (guid != "")
        {
            InstantiateBrandNameSensorPrefabFromGuid(guid, k_OusterIndex);
        }
    }

    [MenuItem("GameObject/Simulation/Prebuilt LiDAR Sensors/Ouster OS0 128", true)]
    static bool ValidateOusterMenuItem()
    {
        var guid = AssetDatabase.FindAssets($"t:Prefab {k_SensorPrefabs[k_OusterIndex]}", null);
        return guid.Length != 0;
    }

    [MenuItem("GameObject/Simulation/Prebuilt LiDAR Sensors/Velodyne Puck")]
    static void CreateVelodyneMenuItem()
    {
        var guid = CheckIfPrefabExists(k_VelodyneIndex);
        if (guid != "")
        {
            InstantiateBrandNameSensorPrefabFromGuid(guid, k_VelodyneIndex);
        }
    }

    [MenuItem("GameObject/Simulation/Prebuilt LiDAR Sensors/Velodyne Puck", true)]
    static bool ValidateVelodyneMenuItem()
    {
        var guid = AssetDatabase.FindAssets($"t:Prefab {k_SensorPrefabs[k_VelodyneIndex]}", null);
        return guid.Length != 0;
    }

    [MenuItem("GameObject/Simulation/Prebuilt LiDAR Sensors/YDLIDAR X4")]
    static void CreateYdLidarMenuItem()
    {
        var guid = CheckIfPrefabExists(k_YDLidarIndex);
        if (guid != "")
        {
            InstantiateBrandNameSensorPrefabFromGuid(guid, k_YDLidarIndex);
        }
    }

    [MenuItem("GameObject/Simulation/Prebuilt LiDAR Sensors/YDLIDAR X4", true)]
    static bool ValidateYdLidarMenuItem()
    {
        var guid = AssetDatabase.FindAssets($"t:Prefab {k_SensorPrefabs[k_YDLidarIndex]}", null);
        return guid.Length != 0;
    }

    static string CheckIfPrefabExists(int prefabIndex)
    {
        var guids = AssetDatabase.FindAssets($"t:Prefab {k_SensorPrefabs[prefabIndex]}", null);
        if (guids.Length == 0)
        {
            Debug.LogError($"Could not find the \"{k_SensorPrefabs[prefabIndex]}\" prefab in the project. Make sure a prefab with this name exists in the project.");
            return "";
        }
        return guids[0];
    }

    static void InstantiateBrandNameSensorPrefabFromGuid(string guid, int prefabIndex)
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(AssetDatabase.GUIDToAssetPath(guid));

        if (prefab == null)
        {
            Debug.LogError($"Sensor '{k_SensorPrefabs[prefabIndex]}' not found in the project. Make sure the prefab's name is correct.");
            return;
        }

        var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab);

        var parent = Selection.activeGameObject;
        if (parent != null)
        {
            instance.transform.SetParent(parent.transform);
            instance.transform.localPosition = Vector3.zero;
            instance.transform.localRotation = Quaternion.identity;
        }

        Undo.RegisterCreatedObjectUndo(instance, $"Instantiate {k_SensorPrefabs[prefabIndex]}");
        Selection.activeGameObject = instance;
    }
}
