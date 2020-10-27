using System;
using System.Text;
using System.Collections.Generic;
using System.Linq;

namespace RosMessageGeneration
{
    public abstract class Message
    {
        /// <summary>
        ///    Read four bytes from an array of bytes and covert to an int32.
        ///     
        ///    Serialized strings or arrays are always sent with the total length in bytes before the bytes
        ///    containing the value. Used by a deserailizer to determine how many bytes should be
        ///    read and deserialized.
        /// </summary>
        /// <param name="data"></param> Raw bytes data
        /// <param name="offset"></param> Where to start reading from the data byte array
        /// <returns> Number of bytes to read to be able to deserialize the following value. </returns>
        public int DeserializeLength(byte[] data, int offset)
        {
            int byteLength = BitConverter.ToInt32 (data, offset);
            return byteLength;
            
        }

        /// <summary>
        ///     Read some number of bytes from byte array and convert to a UTF8 string.
        /// </summary>
        /// <param name="data"></param> Raw bytes data
        /// <param name="offset"></param> Where to start reading from the data byte array
        /// <param name="stringLength"></param> How many bytes to read to deserialize the string
        /// <returns></returns>
        public string DeserializeString(byte[] data, int offset, int stringLength)
        {
            string deserializedString = System.Text.Encoding.UTF8.GetString(data, offset, stringLength);
            return deserializedString;
        }
        
        /// <summary>
        ///     Take an input string and convert it to a byte array while adhering to the convention
        ///     that the first four bytes are used to tell to deserializer how many bytes are needed
        ///     to deserialize the following string value.
        ///     
        /// </summary>
        /// <param name="inputString"></param> The string to serialize
        /// <returns> Byte array of length of string in bytes with the string serialized to bytes. </returns>
        public byte[] SerializeString(string inputString)
        {
            byte[] encodedString = Encoding.ASCII.GetBytes(inputString);
            byte[] encodedPrefix = BitConverter.GetBytes(encodedString.Length);
            var serializedArray = new byte[encodedPrefix.Length + encodedString.Length];
            encodedPrefix.CopyTo(serializedArray, 0);
            encodedString.CopyTo(serializedArray, encodedPrefix.Length);
            return serializedArray;
        }

        /// <summary>
        ///     Get a byte array for each property in the class as a list and flatten the list to a single
        ///     byte array.
        /// </summary>
        /// <param name="omitMessageSize"> Arrays with defined lengths do not need to append a message size length
        ///     since that is defined in the ROS deserializer.</param> 
        /// <returns> Flattened array of bytes </returns>
        public byte[] Serialize(bool omitMessageSize = true)
        {
            var listOfSerializations = SerializationStatements();
            if (!omitMessageSize)
            {
                var messageLength = listOfSerializations.SelectMany(a => a).ToArray().Length;
                listOfSerializations.Insert(0,BitConverter.GetBytes(messageLength));
            }

            byte[] serializedMessage = listOfSerializations.SelectMany(a => a).ToArray();
            return serializedMessage;
        }

        /// <summary>
        ///     For each property in a class call the corresponding Serialize function to create a
        ///     serialized byte array from it's value.
        /// </summary>
        /// <returns> List of all message variables serialized into byte arrays. </returns>
        public abstract List<byte[]> SerializationStatements();

        /// <summary>
        ///     Meant to be used by an already instantiated class to replace it's properties.
        /// </summary>
        /// <param name="data"></param> The bytes to be deserialized
        /// <param name="offset"></param> Where to start reading from the byte array
        /// <returns> The offset which a subsequent Deserialize function can begin reading from. </returns>
        public abstract int Deserialize(byte[] data, int offset);
    }
}