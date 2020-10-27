namespace RosMessageGeneration
{

    public class MessageToken
    {
        public MessageTokenType type;
        public string content;
        public uint lineNum = 0;

        public MessageToken(MessageTokenType type, string content, uint lineNum) {
            this.type = type;
            this.content = content;
            this.lineNum = lineNum;
        }

        public override string ToString() {
            return type + ": " + content + " (" + lineNum + ")";
        }
    }

    public enum MessageTokenType{
        Undefined,
        FilePath,
        Comment,
        BuiltInType,
        DefinedType,
        Header,
        FixedSizeArray,
        VariableSizeArray,
        Identifier,
        ConstantDeclaration,
        Seperator
    }

}