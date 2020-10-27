  

using UnityEngine;

namespace RosSharp.Urdf
{
    [SelectionBase]
    public class UrdfVisual : MonoBehaviour
    {
        [SerializeField]
        public GeometryTypes geometryType;
    }
}