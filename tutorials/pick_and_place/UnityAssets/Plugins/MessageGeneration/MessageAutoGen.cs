using System;
using System.IO;

using System.Collections.Generic;

using System.Linq;

namespace RosMessageGeneration
{

    public class MessageAutoGen
    {
        public static List<string> GenerateSingleMessage(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
            // If no ROS package name is provided, extract from path
            if (rosPackageName.Equals("")) {
                string[] hierarchy = inPath.Split(new char[] { '/', '\\' });
                rosPackageName = hierarchy[hierarchy.Length - 3];
            }

            outPath = Path.Combine(outPath, MsgAutoGenUtilities.ResolvePackageName(rosPackageName));

            string inFileName = Path.GetFileNameWithoutExtension(inPath);

            if (!(rosPackageName.Equals("std_msgs") && (inFileName.Equals("Time") || inFileName.Equals("Duration"))))
            {
                if (verbose) {
                    Console.WriteLine("Parsing: " + inPath);
                    Console.WriteLine("Output Location: " + outPath);
                }

                MessageTokenizer tokenizer = new MessageTokenizer(inPath, new HashSet<string>(MsgAutoGenUtilities.builtInTypesMapping.Keys));
                List<List<MessageToken>> listOfTokens = tokenizer.Tokenize();

                if (listOfTokens.Count != 1)
                {
                    throw new MessageParserException("Unexpected number of sections. Simple message should have 1 section.");
                }

                MessageParser parser = new MessageParser(listOfTokens[0], outPath, rosPackageName, "msg", MsgAutoGenUtilities.builtInTypesMapping, MsgAutoGenUtilities.builtInTypesDefaultInitialValues, MsgAutoGenUtilities.numericTypeDeserializationFunctions, MsgAutoGenUtilities.numericTypeByteSize);
                parser.Parse();
                return parser.GetWarnings();
            }
            else {
                if (verbose) {
                    Console.WriteLine(inFileName + " will not be generated");
                }
                return new List<string>();
            }
        }

        public static List<string> GeneratePackageMessages(string inPath, string outPath, string rosPackageName = "", bool verbose = false) {
            List<string> warnings = new List<string>();

            if (inPath.EndsWith("/") || inPath.EndsWith("\\")) {
                inPath = inPath.Remove(inPath.Length-1);
            }

            if (rosPackageName.Equals("")) {
                rosPackageName = inPath.Split(new char[] { '/', '\\' }).Last();
            }

            string[] files = Directory.GetFiles(Path.Combine(inPath, "msg"), "*.msg");

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No message files found!");
                return warnings;
            }
            else {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " message files.");
                }
                foreach (string file in files) {
                    warnings.AddRange(GenerateSingleMessage(file, outPath, rosPackageName, verbose));
                }
            }
            return warnings;
        }

        public static List<string> GenerateDirectoryMessages(string inPath, string outPath, bool verbose = false) {
            List<string> warnings = new List<string>();

            if (inPath.EndsWith("/") || inPath.EndsWith("\\"))
            {
                inPath = inPath.Remove(inPath.Length - 1);
            }

            string[] files = Directory.GetFiles(inPath, "*.msg", SearchOption.AllDirectories);

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No message files found!");
                return warnings;
            }
            else {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " message files.");
                }
                foreach (string file in files) {
                    warnings.AddRange(GenerateSingleMessage(file, outPath, verbose: verbose));
                }
            }
            return warnings;
        }
    }
}
