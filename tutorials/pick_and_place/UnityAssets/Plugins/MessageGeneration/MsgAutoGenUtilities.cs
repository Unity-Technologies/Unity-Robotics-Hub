using System;

using System.Collections.Generic;

namespace RosMessageGeneration
{
    public class MsgAutoGenUtilities
    {
        public const string ONE_TAB = "    ";
        public const string TWO_TABS = "        ";
        public const string PROPERTY_EXTENSION = " { get; set; }";
        
        public static readonly Dictionary<string, string> builtInTypesMapping = new Dictionary<string, string>
        {
            {"bool", "bool"},
            {"int8", "sbyte"},
            {"uint8", "byte"},
            {"int16", "short"},
            {"uint16", "ushort"},
            {"int32", "int"},
            {"uint32", "uint"},
            {"int64", "long"},
            {"uint64", "ulong"},
            {"float32", "float"},
            {"float64", "double"},
            {"string", "string"},
            {"time", "Time"},
            {"duration", "Duration"},
            {"char", "byte"}, // Deprecated alias for uint8 -> byte  in C#
            {"byte", "sbyte"} // Deprecated alias for int8  -> sbyte in C#
        };

        public static readonly Dictionary<string, string> builtInTypesDefaultInitialValues = new Dictionary<string, string>
        {
            {"bool", "false"},
            {"sbyte", "0"},
            {"byte", "0"},
            {"short", "0"},
            {"ushort", "0"},
            {"int", "0"},
            {"uint", "0"},
            {"long", "0"},
            {"ulong", "0"},
            {"float", "0.0f"},
            {"double", "0.0"},
            {"string", "\"\""},
            {"Time", "new Time()"},
            {"Duration", "new Duration()"}
        };

        public static readonly  Dictionary<string, string> numericTypeDeserializationFunctions = new Dictionary<string, string>
        {
            {"sbyte", "(sbyte)data[offset];"},
            {"byte", "data[offset];"},
            {"bool", "BitConverter.ToBoolean(data, offset)"},
            {"char", "BitConverter.ToChar(data, offset)"},
            {"double", "BitConverter.ToDouble(data, offset)"},
            {"short", "BitConverter.ToInt16(data, offset)"},
            {"int", "BitConverter.ToInt32(data, offset)"},
            {"long", "BitConverter.ToInt64(data, offset)"},
            {"float", "BitConverter.ToSingle(data, offset)"},
            {"ushort", "BitConverter.ToUInt16(data, offset)"},
            {"uint", "BitConverter.ToUInt32(data, offset)"},
            {"ulong", "BitConverter.ToUInt64(data, offset)"}
        };

        public static readonly  Dictionary<string, int> numericTypeByteSize = new Dictionary<string, int>
        {
            {"sbyte", 1},
            {"byte", 1},
            {"bool", 1},
            {"char", 2},
            {"double", 8},
            {"short", 2},
            {"int", 4},
            {"long", 8},
            {"float", 4},
            {"ushort", 2},
            {"uint", 4},
            {"ulong", 8}
        };

        public static string CapitalizeFirstLetter(string s)
        {
            return Char.ToUpper(s[0]) + s.Substring(1);
        }

        public static string LowerFirstLetter(string s) {
            return Char.ToLower(s[0]) + s.Substring(1);
        }

        public static string PascalCase(string s)
        {
            string[] words = s.Split('_');
            for (int i = 0; i < words.Length; i++)
            {
                words[i] = CapitalizeFirstLetter(words[i]);
            }
            return String.Join("", words);

        }

        public static string ResolvePackageName(string s)
        {
            if (s.Contains("_msgs") || s.Contains("_srvs") || s.Contains("_actions"))
            {
                return PascalCase(s.Substring(0, s.LastIndexOf('_')));
            }
            if (s.Contains("_"))
            {
                return PascalCase(s);
            }
            return CapitalizeFirstLetter(s);
        }
    }
}
