using System;
using System.IO;

using System.Collections.Generic;

namespace RosMessageGeneration
{
    public class ServiceAutoGen
    {
        private static readonly string[] types = {"Request", "Response"};

        public static List<string> GenerateSingleService(string inPath, string outPath, string rosPackageName = "", bool verbose = false)
        {
            // If no ROS package name is provided, extract from path
            if (rosPackageName.Equals(""))
            {
                string[] hierarchy = inPath.Split(new char[] { '/', '\\' });
                rosPackageName = hierarchy[hierarchy.Length - 3];
            }

            outPath = Path.Combine(outPath, MsgAutoGenUtilities.ResolvePackageName(rosPackageName));

            string inFileName = Path.GetFileNameWithoutExtension(inPath);

            if (verbose)
            {
                Console.WriteLine("Parsing: " + inPath);
                Console.WriteLine("Output Location: " + outPath);
            }

            MessageTokenizer tokenizer = new MessageTokenizer(inPath, new HashSet<string>(MsgAutoGenUtilities.builtInTypesMapping.Keys));
            List<List<MessageToken>> listsOfTokens = tokenizer.Tokenize();

            if (listsOfTokens.Count != 2)
            {
                throw new MessageParserException("Unexpected number of sections. Service should have 2 sections.");
            }

            List<string> warnings = new List<string>();

            for (int i = 0; i < listsOfTokens.Count; i++)
            {
                List<MessageToken> tokens = listsOfTokens[i];

                // Service is made up of request and response
                string className = inFileName + types[i];

                MessageParser parser = new MessageParser(tokens, outPath, rosPackageName, "srv", MsgAutoGenUtilities.builtInTypesMapping, MsgAutoGenUtilities.builtInTypesDefaultInitialValues, MsgAutoGenUtilities.numericTypeDeserializationFunctions, MsgAutoGenUtilities.numericTypeByteSize, className);
                parser.Parse();
                warnings.AddRange(parser.GetWarnings());
            }
            return warnings;
        }

        public static List<string> GeneratePackageServices(string inPath, string outPath, string rosPackageName = "", bool verbose = false) {
            List<string> warnings = new List<string>();

            string[] files = Directory.GetFiles(Path.Combine(inPath, "srv"), "*.srv");

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No service files found!");
                return warnings;
            }
            else {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " service files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleService(file, outPath, rosPackageName, verbose));
                }
            }

            return warnings;
        }

        public static List<string> GenerateDirectoryServices(string inPath, string outPath, bool verbose = false) {
            List<string> warnings = new List<string>();

            if (inPath.EndsWith("/") || inPath.EndsWith("\\"))
            {
                inPath = inPath.Remove(inPath.Length - 1);
            }

            string[] files = Directory.GetFiles(inPath, "*.srv", SearchOption.AllDirectories);

            if (files.Length == 0)
            {
                Console.Error.WriteLine("No service files found!");
                return warnings;
            }
            else
            {
                if (verbose)
                {
                    Console.WriteLine("Found " + files.Length + " service files.");
                }
                foreach (string file in files)
                {
                    warnings.AddRange(GenerateSingleService(file, outPath, verbose: verbose));
                }
            }
            return warnings;
        }
    }
}
