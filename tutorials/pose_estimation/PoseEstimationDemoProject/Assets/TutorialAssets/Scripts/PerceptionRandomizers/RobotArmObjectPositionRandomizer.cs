using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Experimental.Perception.Randomization.Parameters;
using UnityEngine.Experimental.Perception.Randomization.Randomizers;


[Serializable]
[AddRandomizerMenu("Perception/Robot Arm Object Position Randomizer")]
public class RobotArmObjectPositionRandomizer : InferenceRandomizer
{
    /*  Chooses positions on the plane for placing all objects with the corresponding tag.
     *      - Each object has a radius (defined on the tag, computed per-object based on its bounds)
     *      - No object will be close enough to the edge of the plane to fall off
     *      - All objects will be within the min and max RobotReachability distance to the robot base link (as measured 
     *          on the surface of the plane).
     *      - No object will be close enough to another tagged object to collide with it
     *      
     *  Example use case: placing objects on a table with a robot arm, at random valid positions
     *  where they can be reached by the robot arm. 
     *  
     *  The plane can be manipulated in the editor for easy visualization of the placement surface.
     *  
     *  Assumptions:
     *      - The placement surface is parallel to the global x-z plane. 
     *      - The robot arm is sitting on the placement surface
     */

    public GameObject plane;
    public int maxPlacementTries = 100;

    public GameObject robotBase;
    public float minRobotReachability;
    public float maxRobotReachability;
    public FloatParameter random;  //[0, 1]

    private SurfaceObjectPlacer placer;


    protected override void OnCreate()
    {
        ReachabilityConstraint maxReach = CreateReachabilityConstraint(robotBase.transform.position, maxRobotReachability, ReachabilityConstraint.LimitType.max);
        ReachabilityConstraint minReach = CreateReachabilityConstraint(robotBase.transform.position, minRobotReachability, ReachabilityConstraint.LimitType.min);
        placer = new SurfaceObjectPlacer(plane, random, minReach, maxReach, maxPlacementTries);
    }


    protected override void OnIterationStart()
    {
        OnCustomIteration();
    }

    public override void OnCustomIteration()
    {
        placer.IterationStart();

        IEnumerable<RobotArmObjectPositionRandomizerTag> tags = tagManager.Query<RobotArmObjectPositionRandomizerTag>();


        (List<GameObject> reachableObjects, List<GameObject> otherObjects) = SeparateTags(tags);

        foreach (GameObject reachableObj in reachableObjects)
        {
            bool success = placer.PlaceObject(reachableObj, true);
            if (!success)
            {
                return;
            }
        }

        foreach (GameObject otherObj in otherObjects)
        {
            bool success = placer.PlaceObject(otherObj, false);
            if (!success)
            {
                return;
            }
        }
    }


    // HELPERS

    private (List<GameObject> reachableObjects, List<GameObject> otherObjects) SeparateTags(IEnumerable<RobotArmObjectPositionRandomizerTag> tags)
    {
        List<GameObject> reachableObjects = new List<GameObject>();
        List<GameObject> otherObjects = new List<GameObject>();

        foreach (RobotArmObjectPositionRandomizerTag tag in tags)
        {
            GameObject obj = tag.gameObject;
            if (tag.mustBeReachable)
            {
                reachableObjects.Add(obj);
            }
            else
            {
                otherObjects.Add(obj);
            }
        }
        return (reachableObjects, otherObjects);
    }

    public static ReachabilityConstraint CreateReachabilityConstraint(Vector3 robotBasePosition, float limit, ReachabilityConstraint.LimitType limitType)
    {
        ReachabilityConstraint constraint = new ReachabilityConstraint();
        constraint.robotX = robotBasePosition.x;
        constraint.robotZ = robotBasePosition.z;
        constraint.limitType = limitType;
        constraint.robotReachabilityLimit = limit;
        return constraint;
    }

}