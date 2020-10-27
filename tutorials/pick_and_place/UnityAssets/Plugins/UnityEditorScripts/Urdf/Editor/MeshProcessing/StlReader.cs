  

using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

namespace RosSharp.Urdf
{
    public static class StlReader
    {

        #region Binary

        public static Facet[] ReadBinaryFile(string path)
        {
            Facet[] facets;

            using (FileStream fileStream = new FileStream(path, FileMode.Open, FileAccess.Read))
            {
                using (BinaryReader binaryReader = new BinaryReader(fileStream, new ASCIIEncoding()))
                {
                    binaryReader.ReadBytes(80); // header

                    uint facetCount = binaryReader.ReadUInt32();
                    facets = new Facet[facetCount];

                    for (uint i = 0; i < facetCount; i++)
                        facets[i] = binaryReader.ReadFacet();
                }

                return facets;
            }
        }

        private static Facet ReadFacet(this BinaryReader binaryReader)
        {
            Facet facet = new Facet
            {
                normal = binaryReader.ReadVector3(),
                vertices = new Vector3[3]
            };

            for (int i = 0; i < 3; i++)
                facet.vertices[i] = binaryReader.ReadVector3();

            binaryReader.ReadUInt16(); // padding

            return facet;
        }

        private static Vector3 ReadVector3(this BinaryReader binaryReader)
        {
            Vector3 vector3 = new Vector3();
            for (int i = 0; i < 3; i++)
                vector3[i] = binaryReader.ReadSingle();
            return vector3.Ros2Unity();
        }

        #endregion

        #region Ascii

        public static List<Facet> ReadAsciiFile(string path)
        {
            List<Facet> facets = new List<Facet>();
            Facet facet;
            using (StreamReader streamReader = new StreamReader(path))
                while ((facet = ReadFacet(streamReader)) != null)
                {
                    facets.Add(facet);
                }

            return facets;
        }

        private static Facet ReadFacet(this StreamReader streamReader)
        {
            string line;
            Facet facet = new Facet();
            while ((line = streamReader.ReadLine()) != null)
            {
                line = line.Trim();
                if (line.StartsWith("facet"))
                {
                    facet.normal = ReadVector3(line.Substring(13));
                    facet.vertices = streamReader.GetVertices();
                    return facet;
                }
            }

            return null;
        }

        private static Vector3[] GetVertices(this StreamReader streamReader)
        {
            Vector3[] vertices = new Vector3[3];
            int i = 0;
            string line;
            while ((line = streamReader.ReadLine()) != null)
            {
                line = line.Trim();
                if (line.StartsWith("vertex"))
                    vertices[i++] = ReadVector3(line.Replace("vertex ", ""));
                if (i == 3)
                    return vertices;
            }

            return null;
        }

        private static Vector3 ReadVector3(string _string)
        {
            string[] strings = _string.Trim().Split();

            Vector3 vector3 = new Vector3();

            float.TryParse(strings[0], out vector3.x);
            float.TryParse(strings[1], out vector3.y);
            float.TryParse(strings[2], out vector3.z);

            return vector3.Ros2Unity();
        }

        #endregion

        public class Facet
        {
            public Vector3 normal;
            public Vector3[] vertices;
        }
    }
}