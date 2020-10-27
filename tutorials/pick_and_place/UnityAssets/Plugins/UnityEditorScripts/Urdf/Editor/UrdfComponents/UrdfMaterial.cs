  

using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEditor;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfMaterial
    {
        private const string DefaultMaterialName = "Default";
        private const int RoundDigits = 4;

        public static Dictionary<string, Link.Visual.Material> Materials =
            new Dictionary<string, Link.Visual.Material>();
        
        #region Import
        private static Material CreateMaterial(this Link.Visual.Material urdfMaterial)
        {
            if (urdfMaterial.name == "")
                urdfMaterial.name = GenerateMaterialName(urdfMaterial);

            var material = AssetDatabase.LoadAssetAtPath<Material>(UrdfAssetPathHandler.GetMaterialAssetPath(urdfMaterial.name));
            if (material != null) //material already exists
                return material;

            material = InitializeMaterial();
            if (urdfMaterial.color != null)
            {
                material.color = CreateColor(urdfMaterial.color);
            }
            else if (urdfMaterial.texture != null)
                material.mainTexture = LoadTexture(urdfMaterial.texture.filename);

            AssetDatabase.CreateAsset(material, UrdfAssetPathHandler.GetMaterialAssetPath(urdfMaterial.name));
            return material;
        }

        private static void CreateDefaultMaterial()
        {
            var material = AssetDatabase.LoadAssetAtPath<Material>(UrdfAssetPathHandler.GetMaterialAssetPath(DefaultMaterialName));
            if (material != null)
                return;

            material = InitializeMaterial();
            material.color = new Color(0.33f, 0.33f, 0.33f, 0.0f);

            AssetDatabase.CreateAsset(material, UrdfAssetPathHandler.GetMaterialAssetPath(DefaultMaterialName));
        }

        private static Material InitializeMaterial()
        {
            var material = new Material(Shader.Find("Standard"));
            material.SetFloat("_Metallic", 0.75f);
            material.SetFloat("_Glossiness", 0.75f);
            return material;
        }

        private static string GenerateMaterialName(Link.Visual.Material urdfMaterial)
        {
            var materialName = "";
            if (urdfMaterial.color != null)
            {
                materialName = "rgba-";
                for (var i = 0; i < urdfMaterial.color.rgba.Length; i++)
                {
                    materialName += urdfMaterial.color.rgba[i];
                    if (i != urdfMaterial.color.rgba.Length - 1)
                        materialName += "-";
                }
            }
            else if (urdfMaterial.texture != null)
                materialName = "texture-" + Path.GetFileName(urdfMaterial.texture.filename);

            return materialName;
        }

        private static Color CreateColor(Link.Visual.Material.Color urdfColor)
        {
            return new Color(
                (float)urdfColor.rgba[0],
                (float)urdfColor.rgba[1],
                (float)urdfColor.rgba[2],
                (float)urdfColor.rgba[3]);
        }

        private static Texture LoadTexture(string filename)
        {
            return filename == "" ? null : LocateAssetHandler.FindUrdfAsset<Texture>(filename);
        }


        public static void InitializeRobotMaterials(Robot robot)
        {
            CreateDefaultMaterial();
            foreach (var material in robot.materials)
                CreateMaterial(material);
        }
        
        public static void SetUrdfMaterial(GameObject gameObject, Link.Visual.Material urdfMaterial)
        {
            if (urdfMaterial != null)
            {
                var material = CreateMaterial(urdfMaterial);
                SetMaterial(gameObject, material);
            }
            else
            {
                //If the URDF material is not defined, and the renderer is missing
                //a material, assign the default material.
                Renderer renderer = gameObject.GetComponentInChildren<Renderer>();
                if (renderer != null && renderer.sharedMaterial == null)
                {
                    var defaultMaterial = AssetDatabase.LoadAssetAtPath<Material>(UrdfAssetPathHandler.GetMaterialAssetPath(DefaultMaterialName));
                    SetMaterial(gameObject, defaultMaterial);
                }
            }
        }

        private static void SetMaterial(GameObject gameObject, Material material)
        {
            var renderers = gameObject.GetComponentsInChildren<Renderer>();
            foreach (var renderer in renderers)
                renderer.sharedMaterial = material;
        }
        #endregion

        #region Export

        public static Link.Visual.Material ExportMaterialData(Material material)
        {

            if (material == null) return null;
            if (!Materials.ContainsKey(material.name))
            {
                if (material.mainTexture != null)
                {
                    Link.Visual.Material.Texture texture = ExportTextureData(material.mainTexture);
                    Materials[material.name] = new Link.Visual.Material(material.name, null, texture);
                }
                else if (!material.color.Equals(Color.clear))
                {
                    Link.Visual.Material.Color color = new Link.Visual.Material.Color(ExportRgbaData(material));
                    Materials[material.name] = new Link.Visual.Material(material.name, color);
                }
                else
                    return null;
            }

            return Materials[material.name];
        }

        private static double[] ExportRgbaData(Material material)
        {
            return new double[]
            {
                Math.Round(material.color.r, RoundDigits),
                Math.Round(material.color.g, RoundDigits),
                Math.Round(material.color.b, RoundDigits),
                Math.Round(material.color.a, RoundDigits)
            };
        }

        private static Link.Visual.Material.Texture ExportTextureData(Texture texture)
        {
            string oldTexturePath = UrdfAssetPathHandler.GetFullAssetPath(AssetDatabase.GetAssetPath(texture));
            string newTexturePath = UrdfExportPathHandler.GetNewResourcePath(Path.GetFileName(oldTexturePath));
            if(oldTexturePath != newTexturePath)
                File.Copy(oldTexturePath, newTexturePath, true);

            string packagePath = UrdfExportPathHandler.GetPackagePathForResource(newTexturePath);
            return new Link.Visual.Material.Texture(packagePath);
        }

        #endregion
    }
}