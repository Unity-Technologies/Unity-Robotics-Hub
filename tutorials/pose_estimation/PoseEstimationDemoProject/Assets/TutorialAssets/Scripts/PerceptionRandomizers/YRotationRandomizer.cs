using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Experimental.Perception.Randomization.Parameters;
using UnityEngine.Experimental.Perception.Randomization.Randomizers;

[Serializable]
[AddRandomizerMenu("Perception/Y Rotation Randomizer")]
public class YRotationRandomizer : InferenceRandomizer
{
    public FloatParameter random; // in range (0, 1)

    protected override void OnIterationStart()
    {
        OnCustomIteration();
    }

    public override void OnCustomIteration()
    {
        /* Runs at the start of every iteration. */

        IEnumerable<YRotationRandomizerTag> tags = tagManager.Query<YRotationRandomizerTag>();
        foreach (YRotationRandomizerTag tag in tags)
        {
            float yRotation = random.Sample() * 360.0f;

            // sets rotation
            tag.SetYRotation(yRotation);
        }
    }
}
