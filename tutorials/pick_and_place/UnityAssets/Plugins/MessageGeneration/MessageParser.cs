using System;
using System.IO;
using System.Collections.Generic;

namespace RosMessageGeneration
{
    public class MessageParser
    {

        private List<MessageToken> tokens;

        private readonly string inFilePath;
        private readonly string inFileName;

        private readonly string rosPackageName;
        private readonly string className;
        private readonly string rosMsgName;

        private readonly string outPath;
        private string outFilePath;

        private HashSet<string> imports = new HashSet<string>();

        private readonly Dictionary<string, string> builtInTypeMapping;
        private readonly Dictionary<string, string> builtInTypesDefaultInitialValues;
        private readonly Dictionary<string, string> numericTypeDeserializationFunctions;
        private readonly Dictionary<string, int> numericTypeByteSize;

        private uint lineNum = 1;

        private Dictionary<string, string> symbolTable = new Dictionary<string, string>();
        private HashSet<string> constants = new HashSet<string>();
        private Dictionary<string, int> arraySizes = new Dictionary<string, int>();
            
        private string body = "";

        private List<string> warnings = new List<string>();

        public MessageParser(List<MessageToken> tokens, string outPath, string rosPackageName, string type, Dictionary<string, string> builtInTypeMapping, Dictionary<string, string> builtInTypesDefaultInitialValues, Dictionary<string, string> numericTypeDeserializationFunctions, Dictionary<string, int> numericTypeByteSize, string className = "", string rosMsgName = "") {
            this.tokens = tokens;

            this.inFilePath = tokens[0].content;
            this.inFileName = Path.GetFileNameWithoutExtension(inFilePath);

            this.rosPackageName = rosPackageName;

            if (className.Equals("")) {
                this.className = MsgAutoGenUtilities.CapitalizeFirstLetter(inFileName);
            }
            else
            {
                this.className = className;
            }

            if (rosMsgName.Equals(""))
            {
                this.rosMsgName = MsgAutoGenUtilities.CapitalizeFirstLetter(inFileName);
            }
            else {
                this.rosMsgName = rosMsgName;
            }
            

            this.outPath = outPath;
            this.outFilePath = Path.Combine(outPath, type);

            this.tokens.RemoveAt(0);

            this.builtInTypeMapping = builtInTypeMapping;
            this.builtInTypesDefaultInitialValues = builtInTypesDefaultInitialValues;
            this.numericTypeDeserializationFunctions = numericTypeDeserializationFunctions;
            this.numericTypeByteSize = numericTypeByteSize;
        }

        public void Parse() {
            // If outpath doesn't exist, mkdir
            if (!Directory.Exists(outFilePath))
            {
                Directory.CreateDirectory(outFilePath);
            }
            // Append filename
            this.outFilePath = Path.Combine(this.outFilePath, this.className + ".cs");

            using (StreamWriter writer = new StreamWriter(outFilePath, false))
            {
                // Message -> Lines
                // Lines -> Line Lines | e
                while (!IsEmpty())
                {
                    Line();
                }

                writer.Write(GenerateImports());

                // Write namespace
                writer.Write(
                    "namespace RosMessageTypes." + MsgAutoGenUtilities.ResolvePackageName(rosPackageName) + "\n" +
                    "{\n"
                    );

                // Write class declaration
                writer.Write(
                    MsgAutoGenUtilities.ONE_TAB + "public class " + className + " : Message\n" +
                    MsgAutoGenUtilities.ONE_TAB + "{\n"
                    );

                // Write ROS package name
                writer.Write(MsgAutoGenUtilities.TWO_TABS + "public const string RosMessageName = \"" + rosPackageName + "/" + rosMsgName + "\";\n\n");

                // Write body
                writer.Write(body);
                writer.Write("\n");

                // Write constructors
                writer.Write(GenerateDefaultValueConstructor());
                if (symbolTable.Count != 0 && !new HashSet<string>(symbolTable.Keys).SetEquals(constants)) {
                    writer.Write("\n");
                    writer.Write(GenerateParameterizedConstructor());
                }

                // Write Serializer/Deserializer
                writer.Write(GenerateSerializationStatements());
                writer.Write(GenerateDeserializationStatements());

                // Close class
                writer.Write(MsgAutoGenUtilities.ONE_TAB + "}\n");
                // Close namespace
                writer.Write("}\n");

                writer.Flush();
                writer.Close();
            }
        }

        // Line -> Comment | Declaration
        private void Line() {
            MessageToken peeked = Peek();
            if (PeekType(MessageTokenType.Comment))
            {
                Comment();
            }
            else if (PeekType(MessageTokenType.BuiltInType) || PeekType(MessageTokenType.DefinedType) || PeekType(MessageTokenType.Header))
            {
                Declaration();
            }
            else {
                // Mumble mumble
                if (peeked == null)
                {
                    throw new MessageParserException(
                        "Unexpected end of input " +
                        "' at " + inFilePath + ":" + lineNum);
                }
                else {
                    throw new MessageParserException(
                        "Unexpected token '" + peeked.content +
                        "' at " + inFilePath + ":" + lineNum +
                        ". Expecting a comment or field declaration.");
                }
            }
        }

        // Comment -> # sigma* \n
        private void Comment() {
            body += MsgAutoGenUtilities.TWO_TABS + "// " + MatchByType(MessageTokenType.Comment) + "\n";
        }

        // Declaration -> BuiltInType Identifier | BuiltInType Identifier ConstantDeclaration | BuiltInType ArrayDeclaration Identifier
        // Declaration -> DefinedType Identifier | DefinedType ArrayDeclaration Identifier
        // Declaration -> Header Identifier
        private void Declaration()
        {
            string declaration = "";
            // Type
            MessageToken peeked = Peek();
            string type = "";
            bool canHaveConstDecl = false;
            declaration += MsgAutoGenUtilities.TWO_TABS + "public ";
            if (PeekType(MessageTokenType.BuiltInType)) {
                type = builtInTypeMapping[MatchByType(MessageTokenType.BuiltInType)];
                if (!type.Equals("Time") && !type.Equals("Duration"))
                {
                    // Time and Duration can't have constant declaration
                    // See <wiki.ros.org/msg>
                    canHaveConstDecl = true;
                }
                else {
                    // Need to import Standard
                    imports.Add("Std");
                }
            }
            else if (PeekType(MessageTokenType.DefinedType)) {
                type = MatchByType(MessageTokenType.DefinedType);
                string[] hierarchy = type.Split(new char[] { '/', '\\' });
                // Assume type can only be either:
                // Type
                // package/Type
                switch (hierarchy.Length) {
                    case 1:
                        break;
                    case 2:
                        if (hierarchy[0].Equals("") || hierarchy[1].Equals("")) {
                            throw new MessageParserException(
                            "Invalid field type '" + type + "'. + " +
                            "(" + inFilePath + ":" + lineNum + ")");
                        }
                        string package = MsgAutoGenUtilities.ResolvePackageName(hierarchy[0]);
                        imports.Add(package);
                        type = hierarchy[1];
                        break;
                    default:
                        throw new MessageParserException(
                            "Invalid field type '" + type + "'. + " +
                            "(" + inFilePath + ":" + lineNum + ")");
                }
            } else {
                type = MatchByType(MessageTokenType.Header);
                if (PeekType(MessageTokenType.FixedSizeArray) || PeekType(MessageTokenType.VariableSizeArray)) {
                    Warn(
                        "By convention, there is only one header for each message."  + 
                        "(" + inFilePath + ":" + lineNum + ")");
                }
                if (PeekType(MessageTokenType.Identifier) && !Peek().content.Equals("header")) {
                    Warn(
                        "By convention, a ros message Header will be named 'header'. '" 
                        + Peek().content + "'. (" + inFilePath + ":" + lineNum + ")");
                }
                imports.Add("Std");
            }

            // Array Declaration
            int arraySize = -1;
            if (PeekType(MessageTokenType.FixedSizeArray)) {
                type += "[]";
                canHaveConstDecl = false;
                arraySize = int.Parse(MatchByType(MessageTokenType.FixedSizeArray));
            }
            if (PeekType(MessageTokenType.VariableSizeArray)) {
                type += "[]";
                canHaveConstDecl = false;
                MatchByType(MessageTokenType.VariableSizeArray);
                arraySize = 0;
            }

            // Identifier
            string identifier = MatchByType(MessageTokenType.Identifier);
            // Check for duplicate declaration
            if (symbolTable.ContainsKey(identifier)) {
                throw new MessageParserException(
                    "Field '" + identifier +
                    "' at " + inFilePath + ":" + lineNum +
                    " already declared!");
            }
            // Check if identifier is a ROS message built-in type
            if (builtInTypeMapping.ContainsKey(identifier) && identifier.Equals("time") && identifier.Equals("duration")) {
                throw new MessageParserException(
                    "Invalid field identifier '" + identifier +
                    "' at " + inFilePath + ":" + lineNum +
                    ". '" + identifier + "' is a ROS message built-in type.");
            }

#if NETFRAMEWORK
            CodeDomProvider provider = CodeDomProvider.CreateProvider("C#");
            // Check if identifier is a C# keyword
            if (!provider.IsValidIdentifier(identifier))
            {
                Warn(
                    "'" + identifier + "' is a C# keyword. We have appended \"_\" at the front to avoid C# compile-time issues." +
                    "(" + inFilePath + ":" + lineNum + ")");
                declaration = MsgAutoGenUtilities.TWO_TABS + "[JsonProperty(\"" + identifier + "\")]\n" + declaration; 
                identifier = "_" + identifier;
            }
#else
            Warn(
                "'CodeDomProvider class might not exist on your platform. We did not check whether " + identifier + "' is a C# keyword." +
                "(" + inFilePath + ":" + lineNum + ")");
#endif

            symbolTable.Add(identifier, type);

            // Array declaration table
            if (arraySize > -1) {
                arraySizes.Add(identifier, arraySize);
            }

            // Constant Declaration
            if (PeekType(MessageTokenType.ConstantDeclaration))
            {
                if (canHaveConstDecl)
                {
                    declaration += "const " + type + " " + identifier + " = ";
                    declaration += ConstantDeclaration(type);
                    constants.Add(identifier);
                }
                else
                {
                    throw new MessageParserException(
                        "Type " + type +
                        "' at " + inFilePath + ":" + lineNum +
                        " cannot have constant declaration");
                }
            }
            else {
                declaration += type + " " + identifier + MsgAutoGenUtilities.PROPERTY_EXTENSION + "\n";
            }
            body += declaration;
        }

        // Constant Declaration -> = NumericalConstantValue Comment
        // Constant Declaration -> = StringConstantValue
        // Note that a comment cannot be present in a string constant definition line
        private string ConstantDeclaration(string type) {
            string declaration = MatchByType(MessageTokenType.ConstantDeclaration);
            if (type.Equals("string"))
            {
                return "\"" + declaration.Trim() + "\";\n";
            }
            else {
                string ret = "";
                // Parse constant value using exisiting C# routines
                // Parse by invoking method
                // First check if a comment exists
                string val = "";
                string comment = "";
                if (declaration.Contains("#"))
                {
                    string[] contents = declaration.Split('#');
                    val = contents[0].Trim();
                    comment = String.Join("#", contents, 1, contents.Length - 1);
                }
                else {
                    val = declaration.Trim();
                }
                // Parse value
                switch (type) {
                    case "bool":
                        if (val.Equals("True"))
                        {
                            ret += "true";
                        }
                        else if (val.Equals("False"))
                        {
                            ret += "false";
                        }
                        else {
                            if (byte.TryParse(val, out byte a))
                            {
                                if (a == 0)
                                {
                                    ret += "false";
                                }
                                else
                                {
                                    ret += "true";
                                }
                            }
                            else {
                                throw new MessageParserException(
                                    "Type mismatch: Expecting bool, but value '" + val +
                                    "' at " + inFilePath + ":" + lineNum +
                                    " is not bool/uint8/byte");
                            }
                        }
                        break;
                    case "sbyte":
                        if (sbyte.TryParse(val, out sbyte b))
                        {
                            ret += val;
                        }
                        else {
                            throw new MessageParserException(
                                "Type mismatch: Expecting int8, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not int8/sbyte");
                        }
                        break;
                    case "byte":
                        if (byte.TryParse(val, out byte c))
                        {
                            ret += val;
                        }
                        else {
                            throw new MessageParserException(
                                "Type mismatch: Expecting uint8, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not uint8/byte");
                        }
                        break;
                    case "short":
                        if (short.TryParse(val, out short d))
                        {
                            ret += val;
                        }
                        else {
                            throw new MessageParserException(
                                "Type mismatch: Expecting int16, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not int16/short");
                        }
                        break;
                    case "ushort":
                        if (ushort.TryParse(val, out ushort e))
                        {
                            ret += val;
                        }
                        else
                        {
                            throw new MessageParserException(
                                "Type mismatch: Expecting uint16, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not uint16/ushort");
                        }
                        break;
                    case "int":
                        if (int.TryParse(val, out int f))
                        {
                            ret += val;
                        }
                        else {
                            throw new MessageParserException(
                                "Type mismatch: Expecting int32, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not int32/int");
                        }
                        break;
                    case "uint":
                        if (uint.TryParse(val, out uint g))
                        {
                            ret += val;
                        }
                        else
                        {
                            throw new MessageParserException(
                                "Type mismatch: Expecting uint32, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not uint32/uint");
                        }
                        break;
                    case "long":
                        if (long.TryParse(val, out long h))
                        {
                            ret += val;
                        }
                        else
                        {
                            throw new MessageParserException(
                                "Type mismatch: Expecting int64, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not int64/long");
                        }
                        break;
                    case "ulong":
                        if (ulong.TryParse(val, out ulong i))
                        {
                            ret += val;
                        }
                        else {
                            throw new MessageParserException(
                                "Type mismatch: Expecting uint64, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not uint64/ulong");
                        }
                        break;
                    case "float":
                        if (float.TryParse(val, out float j))
                        {
                            ret += val;
                        }
                        else {
                            throw new MessageParserException(
                                "Type mismatch: Expecting float32, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not float32/float");
                        }
                        break;
                    case "double":
                        if (double.TryParse(val, out double k))
                        {
                            ret += val;
                        }
                        else {
                            throw new MessageParserException(
                                "Type mismatch: Expecting float64, but value '" + val +
                                "' at " + inFilePath + ":" + lineNum +
                                " is not float64/double");
                        }
                        break;
                }
                ret += ";";

                // Take care of comment
                if (!comment.Equals("")) {
                    ret += " // " + comment;
                }

                return ret + "\n";
            }
        }

        private string GenerateImports()
        {
            string importsStr = "";

            importsStr += "using System;\n";
            importsStr += "using System.Collections.Generic;\n";
            importsStr += "using System.Text;\n";
            importsStr += "using RosMessageGeneration;\n";

            if (imports.Count > 0) {
                foreach (string s in imports)
                {
                    importsStr += "using RosMessageTypes." + s + ";\n";
                }
            }
            importsStr += "\n";

            return importsStr;
        }

        private string GenerateDefaultValueConstructor() {
            string constructor = "";

            constructor += MsgAutoGenUtilities.TWO_TABS + "public " + className + "()\n";
            constructor += MsgAutoGenUtilities.TWO_TABS + "{\n";

            foreach (string identifier in symbolTable.Keys) {
                if (!constants.Contains(identifier)) {
                    constructor += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "this." + identifier + " = ";
                    string type = symbolTable[identifier];
                    if (builtInTypesDefaultInitialValues.ContainsKey(type))
                    {
                        constructor += builtInTypesDefaultInitialValues[type];
                    }else if (arraySizes.ContainsKey(identifier))
                    {
                        constructor += "new " + type.Remove(type.Length - 1) + arraySizes[identifier] + "]";
                    }
                    else {
                        constructor += "new " + type + "()";
                    }
                    constructor += ";\n";
                }
            }

            constructor += MsgAutoGenUtilities.TWO_TABS + "}\n";

            return constructor;
        }

        private string GenerateParameterizedConstructor() {
            string constructor = "";

            string parameters = "";
            string assignments = "";

            foreach (string identifier in symbolTable.Keys)
            {
                if (!constants.Contains(identifier))
                {
                    string type = symbolTable[identifier];
                    parameters += type + " " + identifier + ", ";
                    assignments += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "this." + identifier + " = " + identifier + ";\n";
                }
            }

            if (!parameters.Equals("")) {
                parameters = parameters.Substring(0, parameters.Length - 2);
            }
            
            constructor += MsgAutoGenUtilities.TWO_TABS + "public " + className + "(" + parameters + ")\n";
            constructor += MsgAutoGenUtilities.TWO_TABS + "{\n";
            constructor += assignments;
            constructor += MsgAutoGenUtilities.TWO_TABS + "}\n";

            return constructor;
        }

        private string GenerateDeserializationStatements()
        {
            string function = "";
            function += MsgAutoGenUtilities.TWO_TABS + "public override int Deserialize(byte[] data, int offset)\n";
            function += MsgAutoGenUtilities.TWO_TABS + "{\n";

            // Four important types, arrays, strings, classes, and numerics
            foreach (string identifier in symbolTable.Keys)
            {
                 if (!constants.Contains(identifier))
                 {
                     string type = symbolTable[identifier];

                     // Is array type
                     if (arraySizes.ContainsKey(identifier))
                     {
                         // Remove trailing brackets
                         type = type.Substring(0, type.Length-2);
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "\n";
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +
                                     "var "+ identifier + "ArrayLength = DeserializeLength(data, offset);\n";
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "offset += 4;\n";
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "this." + identifier + "= new " + type + "[" + identifier + "ArrayLength];\n";

                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +
                                     "for(var i =0; i <" + identifier + "ArrayLength; i++)\n";
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "{\n";

                         // TODO: Replace with Array.Copy() https://docs.microsoft.com/en-us/dotnet/api/system.array.copy?view=netcore-3.1
                         if (type.Equals("byte"))
                         {
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "this." + identifier + "[i] = data[offset];\n";
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "offset += " + 1 + ";\n";
                         }
                         else if (type.Equals("sbyte"))
                         {
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "this." + identifier + "[i] = (" + type + ")data[offset];\n";
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "offset += " + 1 + ";\n";
                         }
                         else if (type.Equals("string"))
                         {
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "var " + identifier + "StringBytesLength = DeserializeLength(data, offset);\n";
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "offset += 4;\n";

                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "this." +identifier + "[i] = DeserializeString(data, offset, " + identifier + "StringBytesLength);\n";
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "offset += " + identifier + "StringBytesLength;\n";

                         }
                         // Numeric type array
                         else if (numericTypeDeserializationFunctions.ContainsKey(type))
                         {
                             var converterFunction = numericTypeDeserializationFunctions[type];
                             var offsetAddition = numericTypeByteSize[type];

                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "this." + identifier +  "[i] = " + converterFunction + ";\n";
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "offset += " + offsetAddition + ";\n";
                         }
                         // Assume custom message array
                         else
                         {
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "this." + identifier + "[i] = new " + type + "();\n";
                             function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "offset = this." + identifier + "[i].Deserialize(data, offset);\n";
                         }

                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "}\n";
                     }
                     // End of Array logic
                     
                     else if (type.Equals("string"))
                     {
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "var " + identifier + "StringBytesLength = DeserializeLength(data, offset);\n";
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "offset += 4;\n";

                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "this." + identifier + " = DeserializeString(data, offset, " + identifier + "StringBytesLength);\n";
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "offset += " + identifier + "StringBytesLength;\n";

                     }
                     // Message is a custom type and call it's serialization function. Time and duration are special cases
                     else if (!MsgAutoGenUtilities.builtInTypesMapping.ContainsValue(type) || type.Equals("Time") || type.Equals("Duration"))
                     {
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "offset = this." + identifier + ".Deserialize(data, offset);\n";
                     }
                     // Assume numeric type
                     else
                     {
                         var converterFunction = numericTypeDeserializationFunctions[type];
                         var offsetAddition = numericTypeByteSize[type];
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "this." + identifier +  " = " + converterFunction + ";\n";
                         function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "offset += " + offsetAddition + ";\n";
                     }
                 }
            }

            function += "\n" + MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +"return offset;\n";
            function += MsgAutoGenUtilities.TWO_TABS + "}\n\n";

            return function;
        }

        private string GenerateSerializationStatements()
        {
            string function = "";
            function += MsgAutoGenUtilities.TWO_TABS + "public override List<byte[]> SerializationStatements()\n";
            function += MsgAutoGenUtilities.TWO_TABS + "{\n";
            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "var listOfSerializations = new List<byte[]>();\n";

            // Four important types, arrays, strings, classes, and numerics
            foreach (string identifier in symbolTable.Keys)
            {
                if (!constants.Contains(identifier))
                {
                    string type = symbolTable[identifier];
                    // Is array type
                    if (arraySizes.ContainsKey(identifier))
                    {
                        // Remove trailing brackets
                        type = type.Substring(0, type.Length-2);
                        function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "\n";
                        function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +
                                    "listOfSerializations.Add(BitConverter.GetBytes(" + identifier + ".Length));\n";

                        if (type == "byte")
                        {
                            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "listOfSerializations.Add(this." + identifier + ");\n";
                        }
                        else if (type == "sbyte")
                        {
                            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "listOfSerializations.Add((byte[]) (Array)this." + identifier + ");\n";

                        }
                        else if (type.Equals("string"))
                        {
                            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +
                                        "foreach(var entry in " + identifier + ")\n";
                            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS +
                                        "listOfSerializations.Add(SerializeString(entry));\n";
                        }
                        // Assume custom message type with Serialize function
                        else if (!MsgAutoGenUtilities.builtInTypesMapping.ContainsValue(type))
                        {
                            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +
                                        "foreach(var entry in " + identifier + ")\n";
                            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "listOfSerializations.Add(entry.Serialize());\n";
                        }
                        // Assume numeric type
                        else
                        {
                            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +
                                        "foreach(var entry in " + identifier + ")\n";
                            function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.TWO_TABS + "listOfSerializations.Add(BitConverter.GetBytes(entry));\n";
                        }
                    }
                    // End of Array
                    // Strings have a specific function in Message.cs
                    else if (type.Equals("string"))
                    {
                        function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +
                                    "listOfSerializations.Add(SerializeString(this." + identifier + "));\n";
                    }
                    // Message is a custom type and call it's serialization function
                    else if (!MsgAutoGenUtilities.builtInTypesMapping.ContainsValue(type) || type.Equals("Time") || type.Equals("Duration"))
                    {
                        function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB + "listOfSerializations.AddRange(" +identifier + ".SerializationStatements());\n";
                    }
                    // Assume numeric type
                    else
                    {
                        function += MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +
                                    "listOfSerializations.Add(BitConverter.GetBytes(this." + identifier +
                                    "));\n";
                    }
                }
            }

            function += "\n" + MsgAutoGenUtilities.TWO_TABS + MsgAutoGenUtilities.ONE_TAB +"return listOfSerializations;\n";
            function += MsgAutoGenUtilities.TWO_TABS + "}\n\n";
            
            return function;
        }

        private string MatchByType(MessageTokenType type) {
            MessageToken token = tokens[0];
            if (token.type.Equals(type))
            {
                tokens.RemoveAt(0);
                // Update line num
                if (!IsEmpty()) {
                    lineNum = tokens[0].lineNum;
                }
                return token.content;
            }
            else {
                throw new MessageParserException(
                    "Unexpected token '" + token.content + 
                    "' at " + inFilePath + ":" + token.lineNum + 
                    ". Expecting a token of type " + 
                    Enum.GetName(typeof(MessageTokenType), token.type));
            }
        }

        private MessageToken Peek() {
            if (IsEmpty()) {
                return null;
            }
            return tokens[0];
        }

        private bool PeekType(MessageTokenType type) {
            if (IsEmpty()) {
                return false;
            }
            return tokens[0].type.Equals(type);
        }

        private void Warn(string msg) {
            warnings.Add(msg);
        }

        public List<string> GetWarnings() {
            return warnings;
        }

        private bool IsEmpty() {
            return tokens.Count == 0;
        }
    }

    public class MessageParserException : Exception
    {
        public MessageParserException(string msg) : base(msg) { }
    }
}

