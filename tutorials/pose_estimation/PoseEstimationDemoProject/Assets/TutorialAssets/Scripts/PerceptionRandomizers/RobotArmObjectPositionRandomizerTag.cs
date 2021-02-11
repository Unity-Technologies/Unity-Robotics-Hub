using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Perception.Randomization.Randomizers;

[AddComponentMenu("Perception/RandomizerTags/RobotArmObjectPositionRandomizerTag")]
[RequireComponent(typeof(Renderer))]
public class RobotArmObjectPositionRandomizerTag : RandomizerTag
{
    public bool mustBeReachable;
}
