using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Robotics.PickAndPlace;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

[TestFixture, Category("UnitTests")]
public class TargetPlacementTests
{
    const int k_NumAllowedFrames = 5;
    const string k_NamePlaced = "TargetPlacementPlaced";
    const string k_NameOutside = "TargetPlacementOutside";
    const string k_NameFloating = "TargetPlacementFloating";

    [UnitySetUp]
    public IEnumerator LoadSceneAndStartPlayMode()
    {
        SceneManager.LoadScene("TargetPlacementTest");
        yield return new EnterPlayMode();
    }

    [UnityTearDown]
    public IEnumerator ExitSceneOnTearDown()
    {
        yield return new ExitPlayMode();
    }

    public static IEnumerable<TestCaseData> TargetPlacementCases()
    {
        yield return new TestCaseData(k_NamePlaced, TargetPlacement.PlacementState.InsidePlaced).Returns(null);
        yield return new TestCaseData(k_NameOutside, TargetPlacement.PlacementState.Outside).Returns(null);
        yield return new TestCaseData(k_NameFloating, TargetPlacement.PlacementState.InsideFloating).Returns(null);
    }

    [UnityTest, TestCaseSource(nameof(TargetPlacementCases))]
    public IEnumerator TargetPlacement_WithStaticObjects_SetsStateCorrectly(
        string name, TargetPlacement.PlacementState stateExpected)
    {
        var targetPlacement = GameObject.Find(name)?.GetComponent<TargetPlacement>();

        Assert.IsNotNull(targetPlacement, $"Failed to find {name}");

        var numFramesTested = 0;

        while (targetPlacement.CurrentState != stateExpected && numFramesTested < k_NumAllowedFrames)
        {
            numFramesTested++;
            yield return null;
        }

        Assert.AreEqual(targetPlacement.CurrentState, stateExpected);
    }
}
