  

using System.Xml;
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfPlugin : MonoBehaviour
    {
        [TextArea(maxLines: 8, minLines: 4)]
        public string PluginText;

        public static void Create(Transform parent, Plugin plugin = null)
        {
            UrdfPlugin urdfPlugin = parent.gameObject.AddComponent<UrdfPlugin>();
            if (plugin != null)
                urdfPlugin.PluginText = plugin.text;
        }

        public Plugin ExportPluginData()
        {
            if (PluginText == null || PluginText == "") return null;

            try
            {
                XmlDocument xDoc = new XmlDocument();
                xDoc.LoadXml(PluginText);

                return new Plugin(PluginText);
            }
            catch (XmlException e)
            {
                Debug.LogWarning("UrdfPlugin contains invalid XML. The contents of this plugin will not be " +
                                 "written to the URDF file.\nXML Error: " + e.Message, this);
                return null;
            }
        }
    }
}
