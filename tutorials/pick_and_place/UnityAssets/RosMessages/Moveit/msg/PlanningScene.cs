using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class PlanningScene : Message
    {
        public const string RosMessageName = "moveit_msgs-master/PlanningScene";

        //  name of planning scene
        public string name { get; set; }
        //  full robot state
        public RobotState robot_state { get; set; }
        //  The name of the robot model this scene is for
        public string robot_model_name { get; set; }
        // additional frames for duplicating tf (with respect to the planning frame)
        public TransformStamped[] fixed_frame_transforms { get; set; }
        // full allowed collision matrix
        public AllowedCollisionMatrix allowed_collision_matrix { get; set; }
        //  all link paddings
        public LinkPadding[] link_padding { get; set; }
        //  all link scales
        public LinkScale[] link_scale { get; set; }
        //  Attached objects, collision objects, even the octomap or collision map can have 
        //  colors associated to them. This array specifies them.
        public ObjectColor[] object_colors { get; set; }
        //  the collision map
        public PlanningSceneWorld world { get; set; }
        //  Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
        public bool is_diff { get; set; }

        public PlanningScene()
        {
            this.name = "";
            this.robot_state = new RobotState();
            this.robot_model_name = "";
            this.fixed_frame_transforms = new TransformStamped[0];
            this.allowed_collision_matrix = new AllowedCollisionMatrix();
            this.link_padding = new LinkPadding[0];
            this.link_scale = new LinkScale[0];
            this.object_colors = new ObjectColor[0];
            this.world = new PlanningSceneWorld();
            this.is_diff = false;
        }

        public PlanningScene(string name, RobotState robot_state, string robot_model_name, TransformStamped[] fixed_frame_transforms, AllowedCollisionMatrix allowed_collision_matrix, LinkPadding[] link_padding, LinkScale[] link_scale, ObjectColor[] object_colors, PlanningSceneWorld world, bool is_diff)
        {
            this.name = name;
            this.robot_state = robot_state;
            this.robot_model_name = robot_model_name;
            this.fixed_frame_transforms = fixed_frame_transforms;
            this.allowed_collision_matrix = allowed_collision_matrix;
            this.link_padding = link_padding;
            this.link_scale = link_scale;
            this.object_colors = object_colors;
            this.world = world;
            this.is_diff = is_diff;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.AddRange(robot_state.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.robot_model_name));
            
            listOfSerializations.Add(BitConverter.GetBytes(fixed_frame_transforms.Length));
            foreach(var entry in fixed_frame_transforms)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(allowed_collision_matrix.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(link_padding.Length));
            foreach(var entry in link_padding)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(link_scale.Length));
            foreach(var entry in link_scale)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(object_colors.Length));
            foreach(var entry in object_colors)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(world.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.is_diff));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            offset = this.robot_state.Deserialize(data, offset);
            var robot_model_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.robot_model_name = DeserializeString(data, offset, robot_model_nameStringBytesLength);
            offset += robot_model_nameStringBytesLength;
            
            var fixed_frame_transformsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.fixed_frame_transforms= new TransformStamped[fixed_frame_transformsArrayLength];
            for(var i =0; i <fixed_frame_transformsArrayLength; i++)
            {
                this.fixed_frame_transforms[i] = new TransformStamped();
                offset = this.fixed_frame_transforms[i].Deserialize(data, offset);
            }
            offset = this.allowed_collision_matrix.Deserialize(data, offset);
            
            var link_paddingArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.link_padding= new LinkPadding[link_paddingArrayLength];
            for(var i =0; i <link_paddingArrayLength; i++)
            {
                this.link_padding[i] = new LinkPadding();
                offset = this.link_padding[i].Deserialize(data, offset);
            }
            
            var link_scaleArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.link_scale= new LinkScale[link_scaleArrayLength];
            for(var i =0; i <link_scaleArrayLength; i++)
            {
                this.link_scale[i] = new LinkScale();
                offset = this.link_scale[i].Deserialize(data, offset);
            }
            
            var object_colorsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.object_colors= new ObjectColor[object_colorsArrayLength];
            for(var i =0; i <object_colorsArrayLength; i++)
            {
                this.object_colors[i] = new ObjectColor();
                offset = this.object_colors[i].Deserialize(data, offset);
            }
            offset = this.world.Deserialize(data, offset);
            this.is_diff = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

    }
}
