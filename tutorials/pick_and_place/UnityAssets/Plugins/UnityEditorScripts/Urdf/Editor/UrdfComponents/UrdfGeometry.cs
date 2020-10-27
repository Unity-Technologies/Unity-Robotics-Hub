  

using System;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public class UrdfGeometry
    {
        private const int RoundDigits = 6;

        public static Link.Geometry ExportGeometryData(GeometryTypes geometryType, Transform transform, bool isCollisionGeometry = false)
        {
            Link.Geometry geometry = null;
            switch (geometryType)
            {
                case GeometryTypes.Box:
                    geometry = new Link.Geometry(new Link.Geometry.Box(ExportUrdfSize(transform)));
                    break;
                case GeometryTypes.Cylinder:
                    geometry = new Link.Geometry(
                        null,
                        new Link.Geometry.Cylinder(ExportUrdfRadius(transform), ExportCylinderHeight(transform)));
                    break;
                case GeometryTypes.Sphere:
                    geometry = new Link.Geometry(null, null, new Link.Geometry.Sphere(ExportUrdfRadius(transform)));
                    break;
                case GeometryTypes.Mesh:
                    geometry = ExportGeometryMeshData(transform.GetChild(0).gameObject, ExportUrdfSize(transform), isCollisionGeometry);
                    break;
            }

            return geometry;
        }
        
        #region Import Helpers

        public static GeometryTypes GetGeometryType(Link.Geometry geometry)
        {
            if (geometry.box != null)
                return GeometryTypes.Box;
            if (geometry.cylinder != null)
                return GeometryTypes.Cylinder;
            if (geometry.sphere != null)
                return GeometryTypes.Sphere;

            return GeometryTypes.Mesh;
        }

        public static void SetScale(Transform transform, Link.Geometry geometry, GeometryTypes geometryType)
        {
            switch (geometryType)
            {
                case GeometryTypes.Box:
                    transform.localScale =
                        new Vector3((float)geometry.box.size[1], (float)geometry.box.size[2], (float)geometry.box.size[0]);
                    break;
                case GeometryTypes.Cylinder:
                    transform.localScale = new Vector3(
                        (float)geometry.cylinder.radius * 2,
                        (float)geometry.cylinder.length / 2,
                        (float)geometry.cylinder.radius * 2);
                    break;
                case GeometryTypes.Sphere:
                    transform.localScale = new Vector3(
                        (float)geometry.sphere.radius * 2,
                        (float)geometry.sphere.radius * 2,
                        (float)geometry.sphere.radius * 2);
                    break;
                case GeometryTypes.Mesh:
                    if (geometry?.mesh?.scale != null)
                    {
                        Vector3 scale = geometry.mesh.scale.ToVector3().Ros2UnityScale();
                        transform.localScale = Vector3.Scale(transform.localScale, scale);
                        transform.localPosition = Vector3.Scale(transform.localPosition, scale);
                    }
                    break;
            }
        }

        public static Mesh CreateCylinderMesh(Link.Geometry.Cylinder cylinder)
        {
            float height = (float)cylinder.length;
            float bottomRadius = (float)cylinder.radius;
            float topRadius = (float)cylinder.radius;
            int nbSides = 30;
            int nbHeightSeg = 30;

            int nbVerticesCap = nbSides + 1;

            #region Vertices

            // bottom + top + sides
            Vector3[] vertices = new Vector3[nbVerticesCap + nbVerticesCap + nbSides * nbHeightSeg * 2 + 2];
            int vert = 0;
            float _2pi = Mathf.PI * 2f;

            // Bottom cap
            float top = 0.5f * height;
            float bottom = -0.5f * height;
            vertices[vert++] = new Vector3(0f, bottom, 0f);
            while (vert <= nbSides)
            {
                float rad = (float)vert / nbSides * _2pi;
                vertices[vert] = new Vector3(Mathf.Cos(rad) * bottomRadius, bottom, Mathf.Sin(rad) * bottomRadius);
                vert++;
            }

            // Top cap
            vertices[vert++] = new Vector3(0f, top, 0f);
            while (vert <= nbSides * 2 + 1)
            {
                float rad = (float)(vert - nbSides - 1) / nbSides * _2pi;
                vertices[vert] = new Vector3(Mathf.Cos(rad) * topRadius, top, Mathf.Sin(rad) * topRadius);
                vert++;
            }

            // Sides
            int v = 0;
            while (vert <= vertices.Length - 4)
            {
                float rad = (float)v / nbSides * _2pi;
                vertices[vert] = new Vector3(Mathf.Cos(rad) * topRadius, top, Mathf.Sin(rad) * topRadius);
                vertices[vert + 1] = new Vector3(Mathf.Cos(rad) * bottomRadius, bottom, Mathf.Sin(rad) * bottomRadius);
                vert += 2;
                v++;
            }
            vertices[vert] = vertices[nbSides * 2 + 2];
            vertices[vert + 1] = vertices[nbSides * 2 + 3];
            #endregion

            #region Normales

            // bottom + top + sides
            Vector3[] normales = new Vector3[vertices.Length];
            vert = 0;

            // Bottom cap
            while (vert <= nbSides)
            {
                normales[vert++] = Vector3.down;
            }

            // Top cap
            while (vert <= nbSides * 2 + 1)
            {
                normales[vert++] = Vector3.up;
            }

            // Sides
            v = 0;
            while (vert <= vertices.Length - 4)
            {
                float rad = (float)v / nbSides * _2pi;
                float cos = Mathf.Cos(rad);
                float sin = Mathf.Sin(rad);

                normales[vert] = new Vector3(cos, 0f, sin);
                normales[vert + 1] = normales[vert];

                vert += 2;
                v++;
            }
            normales[vert] = normales[nbSides * 2 + 2];
            normales[vert + 1] = normales[nbSides * 2 + 3];
            #endregion

            #region UVs
            Vector2[] uvs = new Vector2[vertices.Length];

            // Bottom cap
            int u = 0;
            uvs[u++] = new Vector2(0.5f, 0.5f);
            while (u <= nbSides)
            {
                float rad = (float)u / nbSides * _2pi;
                uvs[u] = new Vector2(Mathf.Cos(rad) * .5f + .5f, Mathf.Sin(rad) * .5f + .5f);
                u++;
            }

            // Top cap
            uvs[u++] = new Vector2(0.5f, 0.5f);
            while (u <= nbSides * 2 + 1)
            {
                float rad = (float)u / nbSides * _2pi;
                uvs[u] = new Vector2(Mathf.Cos(rad) * .5f + .5f, Mathf.Sin(rad) * .5f + .5f);
                u++;
            }

            // Sides
            int u_sides = 0;
            while (u <= uvs.Length - 4)
            {
                float t = (float)u_sides / nbSides;
                uvs[u] = new Vector3(t, 1f);
                uvs[u + 1] = new Vector3(t, 0f);
                u += 2;
                u_sides++;
            }
            uvs[u] = new Vector2(1f, 1f);
            uvs[u + 1] = new Vector2(1f, 0f);
            #endregion

            #region Triangles
            int nbTriangles = nbSides + nbSides + nbSides * 2;
            int[] triangles = new int[nbTriangles * 3 + 3];

            // Bottom cap
            int tri = 0;
            int i = 0;
            while (tri < nbSides - 1)
            {
                triangles[i] = 0;
                triangles[i + 1] = tri + 1;
                triangles[i + 2] = tri + 2;
                tri++;
                i += 3;
            }
            triangles[i] = 0;
            triangles[i + 1] = tri + 1;
            triangles[i + 2] = 1;
            tri++;
            i += 3;

            // Top cap
            //tri++;
            while (tri < nbSides * 2)
            {
                triangles[i] = tri + 2;
                triangles[i + 1] = tri + 1;
                triangles[i + 2] = nbVerticesCap;
                tri++;
                i += 3;
            }

            triangles[i] = nbVerticesCap + 1;
            triangles[i + 1] = tri + 1;
            triangles[i + 2] = nbVerticesCap;
            tri++;
            i += 3;
            tri++;

            // Sides
            while (tri <= nbTriangles)
            {
                triangles[i] = tri + 2;
                triangles[i + 1] = tri + 1;
                triangles[i + 2] = tri + 0;
                tri++;
                i += 3;

                triangles[i] = tri + 1;
                triangles[i + 1] = tri + 2;
                triangles[i + 2] = tri + 0;
                tri++;
                i += 3;
            }
            #endregion

            Mesh mesh = new Mesh
            {
                vertices = vertices,
                normals = normales,
                uv = uvs,
                triangles = triangles
            };

            mesh.RecalculateBounds();
            return mesh;
        }

        #endregion

        #region Export Helpers

        private static double[] ExportUrdfSize(Transform transform)
        {
            return transform.localScale.Unity2RosScale().ToRoundedDoubleArray();
        }

        private static double ExportUrdfRadius(Transform transform)
        {
            return Math.Round(transform.localScale.x / 2, RoundDigits);
        }

        private static double ExportCylinderHeight(Transform transform)
        {
            return Math.Round(transform.localScale.y * 2, RoundDigits);
        }

        private static Link.Geometry ExportGeometryMeshData(GameObject geometryObject, double[] urdfSize, bool isCollisionGeometry)
        {
            string newFilePath = UrdfMeshExportHandler.CopyOrCreateMesh(geometryObject, isCollisionGeometry);
            string packagePath = UrdfExportPathHandler.GetPackagePathForMesh(newFilePath);

            return new Link.Geometry(null, null, null, new Link.Geometry.Mesh(packagePath, urdfSize));
        }

        public static void CheckForUrdfCompatibility(Transform transform, GeometryTypes type)
        {
            Transform childTransform = transform.GetChild(0);
            if (IsTransformed(childTransform, type))
            {
                Debug.LogWarning("Changes to the transform of " + childTransform.name + " cannot be exported to URDF. " +
                                 "Make any translation, rotation, or scale changes to the parent Visual or Collision object instead.",
                    childTransform);
            }

            if (!transform.HasExactlyOneChild())
                Debug.LogWarning("Only one Geometry element is allowed for each Visual or Collision element. In "
                                 + transform.parent.parent.name + ", move each Geometry into its own Visual or Collision.", transform);
        }

        public static bool IsTransformed(Transform transform, GeometryTypes type)
        {
            return transform.localPosition != Vector3.zero
                   || transform.localScale != Vector3.one
                   || (type != GeometryTypes.Mesh && transform.localRotation != Quaternion.identity);
        }

        #endregion
    }
}