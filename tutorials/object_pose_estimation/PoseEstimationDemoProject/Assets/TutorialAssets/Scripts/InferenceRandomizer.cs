using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Perception.Randomization.Randomizers;
using UnityEngine.Perception.Randomization.Scenarios;

namespace UnityEngine.Perception.Randomization.Randomizers
{
    public abstract class InferenceRandomizer : Randomizer
    {
        public abstract void OnCustomIteration();
    }
}

public class InferenceRandomization : MonoBehaviour
{
    public static void Move(GameObject scenario)
    {
        FixedLengthScenario fixedLenScenario = scenario.GetComponent<FixedLengthScenario>();
        var randomizers = fixedLenScenario.randomizers;
        foreach (InferenceRandomizer rand in randomizers)
        {
            if (rand.enabled == true)
            {
                rand.OnCustomIteration();
            }
        }
    }

}


