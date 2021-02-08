using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Perception.Randomization.Parameters;
using UnityEngine.Perception.Randomization.Randomizers;

[Serializable]
[AddRandomizerMenu("Perception/Light Randomizer")]
public class LightRandomizer : InferenceRandomizer
{
    public FloatParameter lightIntensityParameter; // in range (0.8, 1.2)

    public FloatParameter rotationX; // in range (40, 80)

    public FloatParameter rotationY; // in range (-180, 180)

    public ColorRgbParameter lightColorParameter; //(0.4, 1)

    protected override void OnIterationStart()
    {
        OnCustomIteration();
    }

    public override void OnCustomIteration()
    {
        /*Runs at the start of every iteration*/
        IEnumerable<LightRandomizerTag> tags = tagManager.Query<LightRandomizerTag>();

        foreach (LightRandomizerTag tag in tags)
        {
            var light = tag.gameObject.GetComponent<Light>();
            light.intensity = lightIntensityParameter.Sample();
            light.color = lightColorParameter.Sample();

            Vector3 rotation = new Vector3(rotationX.Sample(), rotationY.Sample(), tag.gameObject.transform.eulerAngles.z);
            tag.gameObject.transform.rotation = Quaternion.Euler(rotation);
        }

    }
}
