// Re-shaders imported URDF meshes onto the active pipeline's Lit shader, so robots do not come in
// magenta. Not part of the SimulationPro package, which is read-only.
//
// Simulation Pro's importer assigns its URP material only when the visual GameObject itself carries a
// MeshRenderer (UrdfRobotFactory.cs:455 and :467). That holds for the .stl path, which calls
// AddComponent<MeshRenderer>() on visualGo. It does not hold for the .dae path (line 433), which
// instantiates the model as a *child* -- so GetComponent<MeshRenderer>() returns null, the assignment
// is skipped, and every Collada visual keeps the Built-in "Standard" materials Unity embedded. The
// Panda's visuals are all .dae.
//
// Converting the materials by hand does not survive a reimport: the .dae meta files carry
// materialLocation: 1 (InPrefab) and materialImportMode: 2, so Unity regenerates them on every import.
// Hooking OnPostprocessModel corrects them as part of that same import instead.
//
// SCOPE: discovered from the project, not hardcoded. Every .urdf under Assets/ is scanned; a model is
// in scope if it is referenced by one of them (package:// URIs resolved) or lives under a folder that
// contains one. Unrelated art assets are untouched.
//
// SELF-HEALING: see UrdfMaterialSelfHeal below. GetVersion() alone is not enough to make this reliable
// when the file is copied between projects -- that is what made this bug reappear here.

using System;
using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace SimProIntegration.EditorTools
{
    /// <summary>
    /// Works out which models in the project belong to a URDF robot, by scanning Assets/ for .urdf
    /// files and reading their mesh references. Replaces the old hardcoded "/UrdfModels/" check, so
    /// robots placed anywhere in the project are covered.
    /// </summary>
    static class UrdfModelScope
    {
        // Mesh paths named explicitly by a .urdf, and folders that contain a .urdf. The folder rule is
        // the fallback for references this resolver cannot follow (unusual package layouts, xacro
        // indirection) so the fixer degrades to "close enough" instead of silently doing nothing.
        static HashSet<string> s_ReferencedModels;
        static List<string> s_UrdfFolders;

        static readonly Regex k_MeshFilename =
            new Regex("filename\\s*=\\s*\"([^\"]+)\"", RegexOptions.Compiled | RegexOptions.IgnoreCase);

        /// <summary>Drops the cached scan, so a newly added .urdf is picked up.</summary>
        internal static void Invalidate()
        {
            s_ReferencedModels = null;
            s_UrdfFolders = null;
        }

        internal static bool Contains(string assetPath)
        {
            if (string.IsNullOrEmpty(assetPath))
                return false;

            EnsureScanned();

            var path = assetPath.Replace('\\', '/');

            if (s_ReferencedModels.Contains(path))
                return true;

            foreach (var folder in s_UrdfFolders)
            {
                if (path.StartsWith(folder, StringComparison.OrdinalIgnoreCase))
                    return true;
            }

            return false;
        }

        /// <summary>All .urdf files in the project, as asset paths. Also used by the menu items.</summary>
        internal static List<string> FindUrdfAssetPaths()
        {
            var results = new List<string>();
            var assetsRoot = AssetsRoot();

            foreach (var full in EnumerateUrdfFiles(assetsRoot))
            {
                var assetPath = ToAssetPath(full, assetsRoot);
                if (assetPath != null)
                    results.Add(assetPath);
            }

            return results;
        }

        // Plain file IO rather than AssetDatabase: this runs from inside OnPostprocessModel, where
        // querying the AssetDatabase is not safe, and in import worker processes where it is not
        // populated the way it is in the main editor.
        static void EnsureScanned()
        {
            if (s_ReferencedModels != null)
                return;

            var referenced = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var folders = new List<string>();
            var assetsRoot = AssetsRoot();

            foreach (var urdfFull in EnumerateUrdfFiles(assetsRoot))
            {
                var urdfPath = urdfFull.Replace('\\', '/');
                var urdfDir = (Path.GetDirectoryName(urdfPath) ?? "").Replace('\\', '/');

                var folderAsset = ToAssetPath(urdfDir, assetsRoot);
                if (string.IsNullOrEmpty(folderAsset))
                    continue;

                // A .urdf sitting directly in Assets/ would put the entire project in scope, which is
                // never what is wanted. Its mesh references are still honoured individually below.
                if (string.Equals(folderAsset, "Assets", StringComparison.OrdinalIgnoreCase))
                {
                    Debug.LogWarning($"[UrdfModelScope] \"{ToAssetPath(urdfPath, assetsRoot)}\" sits in " +
                                     "Assets/ root; not scoping the whole project to it. Move it into its " +
                                     "own folder to have sibling meshes covered.");
                }
                else
                {
                    folders.Add(folderAsset + "/");
                }

                foreach (var reference in ReadMeshReferences(urdfPath))
                {
                    var resolved = ResolveReference(reference, urdfDir, assetsRoot);
                    if (resolved != null)
                        referenced.Add(resolved);
                }
            }

            s_ReferencedModels = referenced;
            s_UrdfFolders = folders;
        }

        static string AssetsRoot() => Application.dataPath.Replace('\\', '/').TrimEnd('/');

        // Formats Unity's ModelImporter handles, which are the only ones OnPostprocessModel fires for.
        static readonly string[] k_ModelExtensions =
        {
            ".dae", ".fbx", ".obj", ".stl", ".blend", ".gltf", ".glb", ".ply", ".3ds", ".dxf"
        };

        static bool IsModelFile(string path)
        {
            foreach (var extension in k_ModelExtensions)
            {
                if (path.EndsWith(extension, StringComparison.OrdinalIgnoreCase))
                    return true;
            }

            return false;
        }

        static IEnumerable<string> EnumerateUrdfFiles(string assetsRoot)
        {
            try
            {
                return Directory.GetFiles(assetsRoot, "*.urdf", SearchOption.AllDirectories);
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
                Debug.LogWarning($"[UrdfModelScope] Could not scan for .urdf files: {e.Message}");
                return Array.Empty<string>();
            }
        }

        static IEnumerable<string> ReadMeshReferences(string urdfFullPath)
        {
            var text = TryReadAllText(urdfFullPath);
            if (text == null)
                yield break;

            foreach (Match match in k_MeshFilename.Matches(text))
                yield return match.Groups[1].Value;
        }

        static string TryReadAllText(string path)
        {
            try
            {
                return File.ReadAllText(path);
            }
            catch (Exception e) when (e is IOException || e is UnauthorizedAccessException)
            {
                Debug.LogWarning($"[UrdfModelScope] Could not read \"{path}\": {e.Message}");
                return null;
            }
        }

        /// <summary>
        /// Turns a URDF mesh reference into an asset path, or null if no such file exists. Handles
        /// "package://pkg/rest", "file://", and plain relative paths. ROS packages have no meaning to
        /// Unity, so the package name is matched against real folders at or above the .urdf -- which
        /// covers both the "urdf next to the package folder" and "urdf in a urdf/ subfolder" layouts.
        /// </summary>
        static string ResolveReference(string reference, string urdfDir, string assetsRoot)
        {
            if (string.IsNullOrEmpty(reference))
                return null;

            var rest = reference.Replace('\\', '/').Trim();

            foreach (var scheme in new[] { "package://", "model://", "file://" })
            {
                if (rest.StartsWith(scheme, StringComparison.OrdinalIgnoreCase))
                {
                    rest = rest.Substring(scheme.Length);
                    break;
                }
            }

            rest = rest.TrimStart('/');
            if (rest.Length == 0)
                return null;

            // filename="" also appears on <texture> in URDF materials; only models matter here.
            if (!IsModelFile(rest))
                return null;

            var slash = rest.IndexOf('/');
            var packageName = slash < 0 ? rest : rest.Substring(0, slash);
            var withinPackage = slash < 0 ? "" : rest.Substring(slash + 1);

            // Walk from the .urdf's own folder upwards, so the nearest match wins.
            var dir = urdfDir;
            while (!string.IsNullOrEmpty(dir) && dir.Length >= assetsRoot.Length)
            {
                // "<dir>/<package>/<rest>" -- the package name is a real folder.
                var candidate = Combine(dir, packageName, withinPackage);
                if (candidate != null && File.Exists(candidate))
                    return ToAssetPath(candidate, assetsRoot);

                // "<dir>/<rest>" -- the package name maps onto this folder itself.
                candidate = Combine(dir, withinPackage, "");
                if (candidate != null && File.Exists(candidate))
                    return ToAssetPath(candidate, assetsRoot);

                // A plain relative path, with no package indirection at all.
                candidate = Combine(dir, rest, "");
                if (candidate != null && File.Exists(candidate))
                    return ToAssetPath(candidate, assetsRoot);

                if (string.Equals(dir, assetsRoot, StringComparison.OrdinalIgnoreCase))
                    break;

                dir = (Path.GetDirectoryName(dir) ?? "").Replace('\\', '/');
            }

            return null;
        }

        // Keeps the root exactly as given -- prefixing a "/" unconditionally would corrupt a
        // Windows path such as "C:/Projects/Robot/Assets".
        static string Combine(string root, string a, string b)
        {
            if (string.IsNullOrEmpty(root))
                return null;

            var result = root.TrimEnd('/');

            foreach (var part in new[] { a, b })
            {
                if (!string.IsNullOrEmpty(part))
                    result += "/" + part.Trim('/');
            }

            // Refuse anything that tries to climb out of the project.
            return result.Contains("..") ? null : result;
        }

        static string ToAssetPath(string absolutePath, string assetsRoot)
        {
            var path = absolutePath.Replace('\\', '/');
            if (!path.StartsWith(assetsRoot, StringComparison.OrdinalIgnoreCase))
                return null;

            return "Assets" + path.Substring(assetsRoot.Length);
        }
    }

    class UrdfUrpMaterialFixer : AssetPostprocessor
    {
        const string k_UrpLitShader = "Universal Render Pipeline/Lit";
        const string k_HdrpLitShader = "HDRP/Lit";

        /// <summary>
        /// The asset pipeline caches model imports and only re-runs a postprocessor when this value
        /// changes. Bump it whenever the conversion logic or the scope rules below change.
        ///
        /// Note what this does NOT do: it does not help a project where the models were already
        /// imported before this file arrived, because a version the pipeline has never seen a
        /// *different* value for is not a change. Copying this file into a fresh project at version 1
        /// is exactly that case, and is why the bug reappeared. UrdfMaterialSelfHeal covers it.
        /// </summary>
        public override uint GetVersion() => 2;

        void OnPostprocessModel(GameObject root)
        {
            if (!UrdfModelScope.Contains(assetPath))
                return;

            var target = GetPipelineLitShader();
            if (target == null)
                return; // Built-in pipeline: the Standard materials are already correct.

            var converted = 0;

            // sharedMaterials rather than materials: during import these are the actual embedded
            // material objects, and we want to mutate them, not spawn per-renderer instances.
            foreach (var renderer in root.GetComponentsInChildren<Renderer>(true))
            {
                foreach (var material in renderer.sharedMaterials)
                {
                    if (ConvertToPipeline(material, target))
                        converted++;
                }
            }

            if (converted > 0)
            {
                Debug.Log($"[UrdfUrpMaterialFixer] {assetPath}: converted {converted} material(s) " +
                          $"to \"{target.name}\".");
            }
        }

        internal static Shader GetPipelineLitShader()
        {
            // currentRenderPipeline, NOT defaultRenderPipeline. A project may assign its URP asset
            // per quality level (Mobile_RPAsset / PC_RPAsset) and leave the Graphics Settings
            // "Default Render Pipeline" slot empty, so defaultRenderPipeline is null even though
            // URP is what actually renders. Reading the wrong one makes this look like a Built-in
            // project and silently skips every conversion.
            //
            // Note: Simulation Pro's own MaterialExtensions.GetRenderPipelineType() reads
            // defaultRenderPipeline and therefore misidentifies such a project as Built-in too --
            // which is why its .stl path also produces Standard (magenta) materials there.
            var pipeline = GraphicsSettings.currentRenderPipeline;
            if (pipeline == null)
                return null;

            var pipelineType = pipeline.GetType().ToString();

            if (pipelineType.Contains("Universal"))
                return Shader.Find(k_UrpLitShader);
            if (pipelineType.Contains("HighDefinition"))
                return Shader.Find(k_HdrpLitShader);

            return null;
        }

        /// <summary>
        /// Re-shaders one material onto the active pipeline's Lit shader, carrying over the
        /// properties whose names differ between Built-in Standard and URP/HDRP Lit.
        /// Returns true if the material was changed.
        /// </summary>
        internal static bool ConvertToPipeline(Material material, Shader target)
        {
            if (material == null || material.shader == null)
                return false;

            if (material.shader == target)
                return false;

            // Read the Built-in properties BEFORE swapping: assigning a new shader drops any
            // property the incoming shader does not declare.
            var color = material.HasProperty("_Color") ? material.GetColor("_Color") : Color.white;
            var mainTex = material.HasProperty("_MainTex") ? material.GetTexture("_MainTex") : null;
            var metallic = material.HasProperty("_Metallic") ? material.GetFloat("_Metallic") : 0f;
            var smoothness = material.HasProperty("_Glossiness") ? material.GetFloat("_Glossiness") : 0.5f;
            var bumpMap = material.HasProperty("_BumpMap") ? material.GetTexture("_BumpMap") : null;

            material.shader = target;

            if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
            if (material.HasProperty("_BaseMap") && mainTex != null) material.SetTexture("_BaseMap", mainTex);
            if (material.HasProperty("_Metallic")) material.SetFloat("_Metallic", metallic);
            if (material.HasProperty("_Smoothness")) material.SetFloat("_Smoothness", smoothness);

            if (material.HasProperty("_BumpMap") && bumpMap != null)
            {
                material.SetTexture("_BumpMap", bumpMap);
                material.EnableKeyword("_NORMALMAP");
            }

            return true;
        }
    }

    /// <summary>
    /// Catches the case GetVersion() cannot: models that were imported and cached before this file
    /// existed in the project. Once per editor session, checks the URDF models actually in the project
    /// and force-reimports only those whose materials are still off-pipeline, which makes the
    /// postprocessor run against them. Cannot loop: a forced reimport converts them, so the next
    /// session finds nothing to do.
    /// </summary>
    static class UrdfMaterialSelfHeal
    {
        const string k_SessionKey = "SimProIntegration.UrdfMaterialSelfHeal.Ran";

        [InitializeOnLoadMethod]
        static void Install()
        {
            // SessionState survives domain reloads but not an editor restart, so this is one pass per
            // editor session rather than one per script recompile.
            if (SessionState.GetBool(k_SessionKey, false))
                return;

            EditorApplication.delayCall += Run;
        }

        static void Run()
        {
            SessionState.SetBool(k_SessionKey, true);

            var target = UrdfUrpMaterialFixer.GetPipelineLitShader();
            if (target == null)
                return; // Built-in pipeline: nothing to correct.

            UrdfModelScope.Invalidate();

            var stale = new List<string>();

            foreach (var guid in AssetDatabase.FindAssets("t:Model"))
            {
                var path = AssetDatabase.GUIDToAssetPath(guid);

                if (!path.StartsWith("Assets/", StringComparison.Ordinal))
                    continue; // Package models are not ours to rewrite.

                if (!UrdfModelScope.Contains(path))
                    continue;

                foreach (var asset in AssetDatabase.LoadAllAssetsAtPath(path))
                {
                    var material = asset as Material;
                    if (material == null || material.shader == null)
                        continue;

                    if (material.shader != target)
                    {
                        stale.Add(path);
                        break;
                    }
                }
            }

            if (stale.Count == 0)
                return;

            try
            {
                AssetDatabase.StartAssetEditing();
                foreach (var path in stale)
                    AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceUpdate);
            }
            finally
            {
                AssetDatabase.StopAssetEditing();
            }

            Debug.Log($"[UrdfMaterialSelfHeal] {stale.Count} URDF model(s) still had off-pipeline " +
                      $"materials and were reimported so they convert to \"{target.name}\". " +
                      "This runs when models were imported before the fixer existed in the project.");
        }
    }

    static class UrdfMeshReimporter
    {
        /// <summary>
        /// Forces a reimport of every model under the selected folder so OnPostprocessModel runs
        /// against meshes that were already imported before this postprocessor existed.
        /// </summary>
        [MenuItem("Assets/Robotics/Reimport URDF Meshes", false, 30)]
        static void ReimportSelected()
        {
            var reimported = 0;

            foreach (var obj in Selection.objects)
            {
                var path = AssetDatabase.GetAssetPath(obj);
                if (string.IsNullOrEmpty(path))
                    continue;

                if (AssetDatabase.IsValidFolder(path))
                {
                    foreach (var guid in AssetDatabase.FindAssets("t:Model", new[] { path }))
                    {
                        AssetDatabase.ImportAsset(
                            AssetDatabase.GUIDToAssetPath(guid), ImportAssetOptions.ForceUpdate);
                        reimported++;
                    }
                }
                else
                {
                    AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceUpdate);
                    reimported++;
                }
            }

            AssetDatabase.Refresh();
            Debug.Log($"[UrdfMeshReimporter] Force-reimported {reimported} asset(s).");
        }

        [MenuItem("Assets/Robotics/Reimport URDF Meshes", true)]
        static bool ReimportSelectedValidate() => Selection.objects.Length > 0;

        /// <summary>
        /// Reimports every model the project's .urdf files point at, wherever they live. Use this
        /// after changing the conversion logic, or if a robot still comes in magenta.
        /// </summary>
        [MenuItem("Tools/Robotics/Reimport All URDF Meshes", false, 30)]
        static void ReimportAll()
        {
            UrdfModelScope.Invalidate();

            var urdfs = UrdfModelScope.FindUrdfAssetPaths();
            var models = new List<string>();

            foreach (var guid in AssetDatabase.FindAssets("t:Model"))
            {
                var path = AssetDatabase.GUIDToAssetPath(guid);

                if (path.StartsWith("Assets/", StringComparison.Ordinal) && UrdfModelScope.Contains(path))
                    models.Add(path);
            }

            if (models.Count == 0)
            {
                Debug.LogWarning($"[UrdfMeshReimporter] Found {urdfs.Count} .urdf file(s) but no models " +
                                 "in scope. Check that the mesh files sit under the .urdf's folder.");
                return;
            }

            try
            {
                AssetDatabase.StartAssetEditing();
                foreach (var path in models)
                    AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceUpdate);
            }
            finally
            {
                AssetDatabase.StopAssetEditing();
            }

            Debug.Log($"[UrdfMeshReimporter] Force-reimported {models.Count} model(s) " +
                      $"from {urdfs.Count} .urdf file(s).");
        }
    }
}
