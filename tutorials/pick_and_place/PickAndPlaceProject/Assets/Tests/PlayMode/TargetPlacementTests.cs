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
    const int k_NumAllowedFramesStatic = 5;
    const int k_NumAllowedFramesDynamic = 20;
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

    TargetPlacement GetTargetPlacement(string name)
    {
        var targetPlacement = GameObject.Find(name)?.GetComponent<TargetPlacement>();
        Assert.IsNotNull(targetPlacement, $"Failed to find {name}");
        return targetPlacement;
    }

    static IEnumerator WaitForState(
        TargetPlacement targetPlacement, TargetPlacement.PlacementState stateExpected, int numFramesToWait)
    {
        var numFramesTested = 0;

        while (targetPlacement.CurrentState != stateExpected && numFramesTested < numFramesToWait)
        {
            numFramesTested++;
            yield return null;
        }

        Assert.AreEqual(stateExpected, targetPlacement.CurrentState);
    }

    [UnityTest, TestCaseSource(nameof(TargetPlacementCases))]
    public IEnumerator TargetPlacement_WithStaticObjects_SetsStateCorrectly(
        string name, TargetPlacement.PlacementState stateExpected)
    {
        var targetPlacement = GetTargetPlacement(name);

        yield return WaitForState(targetPlacement, stateExpected, k_NumAllowedFramesStatic);
    }

    [UnityTest]
    public IEnumerator TargetPlacement_WithFallingObject_ChangesStateCorrectly()
    {
        const string name = "TargetPlacementStateChange";
        var targetPlacement = GetTargetPlacement(name);

        Assert.AreEqual(TargetPlacement.PlacementState.Outside, targetPlacement.CurrentState,
            $"{name} should start with no Target in its bounds.");

        // Target should fall into placement and come to reset
        yield return WaitForState(
            targetPlacement, TargetPlacement.PlacementState.InsideFloating, k_NumAllowedFramesDynamic);
        yield return WaitForState(
            targetPlacement, TargetPlacement.PlacementState.InsidePlaced, k_NumAllowedFramesDynamic);
    }
}
