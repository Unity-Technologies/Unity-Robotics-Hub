   

using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;

namespace RosSharp.Urdf
{
	public class StlWriter
	{
	    private readonly string exportPath;
	    private List<Mesh> meshes;
	    private readonly bool convertToRightHandedCoordinates;

        public enum FileType
	    {
	        Ascii,
	        Binary
	    }
        public static FileType fileType;

        public StlWriter(string exportPath, List<Mesh> meshes, bool convertToRightHandedCoordinates = true)
        {
            this.exportPath = exportPath;
            this.meshes = meshes;
            this.convertToRightHandedCoordinates = convertToRightHandedCoordinates;
        }

		public bool WriteFile()
		{
		    try
		    {
		        if (fileType == FileType.Binary)
		            WriteToBinaryFile();
		        else
		            WriteToAsciiFile();

		        return true;
		    }
		    catch (System.Exception e)
		    {
		        Debug.LogError(e.ToString());
		        return false;
		    }
		}

	    private void WriteToBinaryFile()
	    {
	        using (BinaryWriter writer = new BinaryWriter(File.Open(exportPath, FileMode.Create), new ASCIIEncoding()))
	        {
                //80 byte header
	            writer.Write(new byte[80]);

	            uint totalTriangleCount = (uint) (meshes.Sum(x => x.triangles.Length) / 3);
	            writer.Write(totalTriangleCount);

	            foreach (Mesh mesh in meshes)
	                WriteBinaryMesh(mesh, writer);
	        }
	    }

        private void WriteBinaryMesh(Mesh mesh, BinaryWriter writer)
	    {
	        Vector3[] v = convertToRightHandedCoordinates ? Left2Right(mesh.vertices) : mesh.vertices;
	        Vector3[] n = convertToRightHandedCoordinates ? Left2Right(mesh.normals) : mesh.normals;
	        int[] t = mesh.triangles;
	        int triangleCount = t.Length;

	        if (convertToRightHandedCoordinates)
	            System.Array.Reverse(t);

	        for (int i = 0; i < triangleCount; i += 3)
	        {
	            int a = t[i], b = t[i + 1], c = t[i + 2];

	            Vector3 avg = AverageNormal(n[a], n[b], n[c]);

	            writer.Write(avg.x);
	            writer.Write(avg.y);
	            writer.Write(avg.z);

	            writer.Write(v[a].x);
	            writer.Write(v[a].y);
	            writer.Write(v[a].z);

	            writer.Write(v[b].x);
	            writer.Write(v[b].y);
	            writer.Write(v[b].z);

	            writer.Write(v[c].x);
	            writer.Write(v[c].y);
	            writer.Write(v[c].z);

	            // specification says attribute byte count should be set to 0.
	            writer.Write((ushort)0);
            }
	    }

	    private void WriteToAsciiFile()
	    {
	        string model = BuildAsciiString();
	        File.WriteAllText(exportPath, model);
	    }
        
	    private string BuildAsciiString()
		{
			StringBuilder sb = new StringBuilder();

			string name = meshes.Count == 1 ? meshes[0].name : "Composite Mesh";

			sb.AppendLine($"solid {name}");

			foreach(Mesh mesh in meshes)
			{
				Vector3[] v = convertToRightHandedCoordinates ? Left2Right(mesh.vertices) : mesh.vertices;
				Vector3[] n = convertToRightHandedCoordinates ? Left2Right(mesh.normals) : mesh.normals;
				int[] t = mesh.triangles;
				if(convertToRightHandedCoordinates) System.Array.Reverse(t);
				int triLen = t.Length;

				for(int i = 0; i < triLen; i+=3)
				{
					int a = t[i];
					int b = t[i+1];
					int c = t[i+2];

					Vector3 nrm = AverageNormal(n[a], n[b], n[c]);

				    sb.AppendLine($"facet normal {nrm.x} {nrm.y} {nrm.z}");

				    sb.AppendLine("outer loop");

				    sb.AppendLine($"\tvertex {v[a].x} {v[a].y} {v[a].z}");
				    sb.AppendLine($"\tvertex {v[b].x} {v[b].y} {v[b].z}");
				    sb.AppendLine($"\tvertex {v[c].x} {v[c].y} {v[c].z}");

				    sb.AppendLine("endloop");

				    sb.AppendLine("endfacet");
                }
			}

			sb.AppendLine($"endsolid {name}");

			return sb.ToString();
		}

	    private static Vector3[] Left2Right(Vector3[] v)
	    {
	        Vector3[] r = new Vector3[v.Length];
            for (int i = 0; i < v.Length; i++)
	            r[i] = v[i].Unity2Ros();
	        return r;
	    }

        private static Vector3 AverageNormal(Vector3 a, Vector3 b, Vector3 c)
		{
			return new Vector3(
				(a.x + b.x + c.x) / 3f,
				(a.y + b.y + c.y) / 3f,
				(a.z + b.z + c.z) / 3f );
		}
	}
}
