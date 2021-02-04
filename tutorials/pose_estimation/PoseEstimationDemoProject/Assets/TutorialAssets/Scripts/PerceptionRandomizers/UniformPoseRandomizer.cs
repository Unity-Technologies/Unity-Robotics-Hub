using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Experimental.Perception.Randomization.Parameters;
using UnityEngine.Experimental.Perception.Randomization.Randomizers;

[Serializable]
[AddRandomizerMenu("Perception/Uniform Pose Randomizer")]
public class UniformPoseRandomizer : InferenceRandomizer
{
    /*  Randomizes object's position and rotation relative to a fixed starting pose, over the specified range.
     *  
     *  Example use-case:
     *  Make very small random adjustments to the camera's pose, to make the learned model more robust
     *  to small inaccuracies in placing the real camera.
     */

    public float positionRange = 0.005f;
    public float rotationRangeDegrees = 1.0f; 

    public FloatParameter random; //(-1, 1)

    protected override void OnIterationStart()
    {
        OnCustomIteration();
    }

    public override void OnCustomIteration()
    {
        IEnumerable<UniformPoseRandomizerTag> tags = tagManager.Query<UniformPoseRandomizerTag>();
        foreach (UniformPoseRandomizerTag tag in tags)
        {
            Vector3 adjustedPosition = AdjustedVector(tag.rootPosePosition, positionRange);
            Vector3 adjustedRotation = AdjustedVector(tag.rootPoseRotation, rotationRangeDegrees);
            tag.gameObject.transform.position = adjustedPosition;
            tag.gameObject.transform.eulerAngles = adjustedRotation;
        }
    }

    // HELPERS

    private Vector3 AdjustedVector(Vector3 rootVector, float range)
    {
        float x = AdjustedValue(rootVector.x, range);
        float y = AdjustedValue(rootVector.y, range);
        float z = AdjustedValue(rootVector.z, range);
        Vector3 adjustedVector = new Vector3(x, y, z);
        return adjustedVector;
    }

    private float AdjustedValue(float rootValue, float range)
    {
        float change = range * random.Sample();
        float adjustedVal = rootValue + change;
        return adjustedVal;
    }

}