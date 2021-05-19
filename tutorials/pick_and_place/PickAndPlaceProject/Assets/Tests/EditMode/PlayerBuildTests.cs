using System;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.TestTools;
using UnityEngine;
using UnityEngine.TestTools;

namespace BuildTests
{
    [TestFixture, Explicit, Category("BuildTests")]
    public class PlayerBuilder
    {
        List<EditorBuildSettingsScene> m_EditorBuildSettingsScenes = new List<EditorBuildSettingsScene>();
        BuildSummary m_Summary;
        string m_BuildPath = "Build";

        [UnityPlatform(RuntimePlatform.WindowsEditor)]
        [RequirePlatformSupport(BuildTarget.StandaloneWindows64)]
        [Test]
        public void BuildPlayerStandaloneWindows64()
        {
            BuildPlayer(BuildTargetGroup.Standalone, BuildTarget.StandaloneWindows64, m_BuildPath, BuildOptions.None, out _, out m_Summary);
            Assert.AreEqual(BuildResult.Succeeded, m_Summary.result, " BuildTarget.StandaloneWindows64 failed to build");
        }

        [RequirePlatformSupport(BuildTarget.StandaloneLinux64)]
        [Test]
        public void BuildPlayerLinux()
        {
            BuildPlayer(BuildTargetGroup.Standalone, BuildTarget.StandaloneLinux64, m_BuildPath, BuildOptions.None, out _, out m_Summary);
            Assert.AreEqual(BuildResult.Succeeded, m_Summary.result, "BuildTarget.StandaloneLinux64 failed to build");
        }

        [UnityPlatform(RuntimePlatform.OSXEditor)]
        [RequirePlatformSupport(BuildTarget.StandaloneOSX)]
        [Test]
        public void BuildPlayerOSX()
        {
            BuildPlayer(BuildTargetGroup.Standalone, BuildTarget.StandaloneOSX, m_BuildPath, BuildOptions.None, out _, out m_Summary);
            Assert.AreEqual(BuildResult.Succeeded, m_Summary.result, "BuildTarget.StandaloneLinux64 failed to build");
        }

        void BuildPlayer(BuildTargetGroup buildTargetGroup, BuildTarget buildTarget, string buildOutputPath, BuildOptions buildOptions,
            out BuildReport buildReport, out BuildSummary buildSummary)
        {
            BuildPlayerOptions buildPlayerOptions = new BuildPlayerOptions();
            buildPlayerOptions.locationPathName = buildOutputPath;
            buildPlayerOptions.target = buildTarget;
            buildPlayerOptions.options = buildOptions;
            buildPlayerOptions.targetGroup = buildTargetGroup;

            buildReport = BuildPipeline.BuildPlayer(buildPlayerOptions);
            buildSummary = buildReport.summary;
        }
    }
}
