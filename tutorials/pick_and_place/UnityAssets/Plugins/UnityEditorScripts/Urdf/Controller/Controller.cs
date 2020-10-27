using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.Control
{
    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
    public enum ControlType { PositionControl};

    public class Controller : MonoBehaviour
    {

        private ArticulationBody[] articulationChain;
        private Color[] prevColor;
        private int previousIndex;

        public ControlType control = ControlType.PositionControl;
        public int selectedIndex;
        public string jointName;
        public float stiffness;
        public float damping;
        public float R, G, B, Alpha;
        public float speed = 5f; // Units: degree/s
        public float torque = 100f; // Units: Nm or N
        public float acceleration = 5f;// Units: m/s^2 / degree/s^2


        void Start()
        {
            previousIndex = selectedIndex = 1;
            this.gameObject.AddComponent<FKRobot>();
            articulationChain = this.GetComponentsInChildren<ArticulationBody>();
            foreach (ArticulationBody joint in articulationChain)
            {
                joint.gameObject.AddComponent<JointControl>();
                joint.jointFriction = 10;
                joint.angularDamping = 10;
            }
            jointName = articulationChain[selectedIndex].name;
            StoreColors(selectedIndex);
            B = G = 0;
            Alpha = R = 1;
        }

        void Update()
        {
            bool SelectionInput1 = Input.GetKeyDown("right");
            bool SelectionInput2 = Input.GetKeyDown("left");

            UpdateDirection(selectedIndex);

            if (SelectionInput2)
            {
                if (selectedIndex == 1)
                {
                    selectedIndex = articulationChain.Length - 1;
                }
                else
                {
                    selectedIndex = selectedIndex - 1;
                }
                Highlight(selectedIndex);
            }
            else if (SelectionInput1)
            {
                if (selectedIndex == articulationChain.Length - 1)
                {
                    selectedIndex = 1;
                }
                else
                {
                    selectedIndex = selectedIndex + 1;
                }
                Highlight(selectedIndex);
            }
                
            UpdateDirection(selectedIndex);
        }

        /// <summary>
        /// Highlights the color of the robot by changing the color of the part to a color set by the user in the inspector window
        /// </summary>
        /// <param name="selectedIndex">Index of the link selected in the Articulation Chain</param>
        private void Highlight(int selectedIndex)
        {
            if(selectedIndex == previousIndex)
            {
                return;
            }

            Renderer[] previousMaterialList = articulationChain[previousIndex].transform.GetChild(0).GetComponentsInChildren<Renderer>();

            for (int counter = 0; counter < previousMaterialList.Length; counter++)
            {
                previousMaterialList[counter].material.color = prevColor[counter];
            }
            jointName = articulationChain[selectedIndex].name;
            Renderer[] materialList = articulationChain[selectedIndex].transform.GetChild(0).GetComponentsInChildren<Renderer>();

            StoreColors(selectedIndex);

            foreach (var mesh in materialList)
            {
                Color tempColor = new Color(R, G, B, Alpha);
                mesh.material.color = tempColor;
            }

        }

        /// <summary>
        /// Sets the direction of movement of the joint on every update
        /// </summary>
        /// <param name="jointIndex">Index of the link selected in the Articulation Chain</param>
        private void UpdateDirection(int jointIndex)
        {
            float moveDirection = Input.GetAxis("Vertical");
            JointControl current = articulationChain[jointIndex].GetComponent<JointControl>();            
            if (previousIndex != jointIndex)
            {
                JointControl previous = articulationChain[previousIndex].GetComponent<JointControl>();            
                previous.direction = RotationDirection.None;
                previousIndex = jointIndex;
            }

            if (current.controltype != control)
                UpdateControlType(current);

            if (moveDirection > 0)
            {
                current.direction = RotationDirection.Positive;
            }
            else if (moveDirection < 0)
            {
                current.direction = RotationDirection.Negative;
            }
            else
            {
                current.direction = RotationDirection.None;
            }


        }

        /// <summary>
        /// Stores original color of the part being highlighted
        /// </summary>
        /// <param name="index">Index of the part in the Articulation chain</param>
        private void StoreColors(int index)
        {
            Renderer[] materialLists = articulationChain[index].transform.GetChild(0).GetComponentsInChildren<Renderer>();
            prevColor = new Color[materialLists.Length];
            for (int counter = 0; counter < materialLists.Length; counter++)
            {
                prevColor[counter] = materialLists[counter].sharedMaterial.GetColor("_Color");
            }
        }

        public void UpdateControlType(JointControl joint)
        {
            joint.controltype = control;
            if(control == ControlType.PositionControl)
            {
                ArticulationDrive drive = joint.joint.xDrive;
                drive.stiffness = stiffness;
                drive.damping = 0;
                joint.joint.xDrive = drive;
            }
        }
    }
}
