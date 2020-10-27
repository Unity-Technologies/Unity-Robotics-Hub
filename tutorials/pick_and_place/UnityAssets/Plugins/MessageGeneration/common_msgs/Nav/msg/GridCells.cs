using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Nav
{
    public class GridCells : Message
    {
        public const string RosMessageName = "nav_msgs/GridCells";

        // an array of cells in a 2D grid
        public Header header { get; set; }
        public float cell_width { get; set; }
        public float cell_height { get; set; }
        public Point[] cells { get; set; }

        public GridCells()
        {
            this.header = new Header();
            this.cell_width = 0.0f;
            this.cell_height = 0.0f;
            this.cells = new Point[0];
        }

        public GridCells(Header header, float cell_width, float cell_height, Point[] cells)
        {
            this.header = header;
            this.cell_width = cell_width;
            this.cell_height = cell_height;
            this.cells = cells;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.cell_width));
            listOfSerializations.Add(BitConverter.GetBytes(this.cell_height));
            
            listOfSerializations.Add(BitConverter.GetBytes(cells.Length));
            foreach(var entry in cells)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.cell_width = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.cell_height = BitConverter.ToSingle(data, offset);
            offset += 4;
            
            var cellsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.cells= new Point[cellsArrayLength];
            for(var i =0; i <cellsArrayLength; i++)
            {
                this.cells[i] = new Point();
                offset = this.cells[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
