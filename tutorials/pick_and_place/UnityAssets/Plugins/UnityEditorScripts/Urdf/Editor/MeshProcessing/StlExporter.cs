   

using UnityEngine;
using System.Collections.Generic;

namespace RosSharp.Urdf
{
    public class StlExporter
    {
        private readonly bool exportCollision;
        private readonly string exportPath;
        private readonly GameObject gameObject;

        public StlExporter(string path, GameObject toExport, bool collision = false)
        {
            exportPath = path;
            gameObject = toExport;
            exportCollision = collision;
        }

		public bool Export()
		{
		    List<Mesh> meshes = CreateWorldSpaceMeshes();
			bool success = false;

			if(meshes != null && meshes.Count > 0)
			{
				if(!(exportPath == "" || exportPath == null))
					success = new StlWriter(exportPath, meshes).WriteFile();
			}

			for(int i = 0; meshes != null && i < meshes.Count; i++)
				Object.DestroyImmediate(meshes[i]);

			return success;
		}

		private List<Mesh> CreateWorldSpaceMeshes()
		{
		    //Create a clone with no scale, rotation or transform, so that mesh will be
		    //at original size and position when exported.
		    GameObject clone = Object.Instantiate(gameObject, Vector3.zero, Quaternion.identity);
		    clone.name = gameObject.name;
		    clone.transform.localScale = Vector3.one;
            
		    GameObject root = new GameObject();
		    clone.transform.SetParent(root.transform, true);

		    List<Mesh> meshes = new List<Mesh>();
            if(exportCollision)
            {
                MeshCollider[] meshColliders = root.GetComponentsInChildren<MeshCollider>();

                foreach (MeshCollider meshCollider in meshColliders)
                {
                    if(meshCollider.sharedMesh != null)
                        meshes.Add(TransformMeshToWorldSpace(meshCollider.transform, meshCollider.sharedMesh));
                }
            }
            else
            {
                MeshFilter[] meshFilters = root.transform.GetComponentsInChildren<MeshFilter>();

                foreach (MeshFilter meshFilter in meshFilters)
                {
                    if(meshFilter.sharedMesh != null)
                        meshes.Add(TransformMeshToWorldSpace(meshFilter.transform, meshFilter.sharedMesh));
                }
            }

            Object.DestroyImmediate(root);

			return meshes;
		}

        private static Mesh TransformMeshToWorldSpace(Transform meshTransform, Mesh sharedMesh)
        {
            Vector3[] v = sharedMesh.vertices;
            Vector3[] n = sharedMesh.normals;

            for (int it = 0; it < v.Length; it++)
            {
                v[it] = meshTransform.TransformPoint(v[it]);
                n[it] = meshTransform.TransformDirection(n[it]);
            }

            return new Mesh
            {
                name = meshTransform.name,
                vertices = v,
                normals = n,
                triangles = sharedMesh.triangles
            };
        }
	}
}
