  

using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Linq;
using System.Globalization;

namespace RosSharp.Urdf
{
    public static class XAttributeExtensions
    {
        public static double[] ReadDoubleArray(this XAttribute attribute)
        {
            return Array.ConvertAll(
                ((string)attribute).Split(' ').Where(x => !string.IsNullOrEmpty(x)).ToArray(),
                i => Convert.ToDouble(i, CultureInfo.InvariantCulture));
        }

        public static double ReadOptionalDouble(this XAttribute attribute)
        {
            return (attribute != null) ? (double)attribute : double.NaN;
        }

        public static string DoubleArrayToString(this IEnumerable<double> arr)
        {
            string arrString = arr.Aggregate("", (current, num) => (current + " " + num));
            return arrString.Substring(1); //Gets rid of extra space at start of string
        }
    }
}
