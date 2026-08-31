using System;
using System.Text;
using UnityEngine;

namespace TeleopBridge.Unity
{
    [Serializable]
    public sealed class TeleimagerFrameMetadata
    {
        public string protocol;
        public int version;
        public string stream;
        public long sequence;
        public long capture_timestamp_ms;
        public long publish_timestamp_ms;
        public double sensor_timestamp_ms;
        public int width;
        public int height;
        public string codec;
    }

    /// <summary>
    /// Reads Teleimager v2 timing metadata embedded in JPEG APP15.
    /// The JPEG remains standards-compliant, so existing Texture2D.LoadImage
    /// and video paths can continue to consume the original byte array.
    /// </summary>
    public static class TeleimagerTimestampMetadata
    {
        private static readonly byte[] Magic = Encoding.ASCII.GetBytes("TELEIMAGER\0");

        public static bool TryParse(byte[] jpeg, out TeleimagerFrameMetadata metadata)
        {
            metadata = null;
            if (jpeg == null || jpeg.Length < 6 || jpeg[0] != 0xFF || jpeg[1] != 0xD8)
                return false;

            int offset = 2;
            while (offset + 4 <= jpeg.Length && jpeg[offset] == 0xFF)
            {
                byte marker = jpeg[offset + 1];
                if (marker == 0xDA) // Start of scan
                    break;
                if (marker == 0xD8 || marker == 0xD9)
                {
                    offset += 2;
                    continue;
                }

                int segmentLength = (jpeg[offset + 2] << 8) | jpeg[offset + 3];
                int segmentEnd = offset + 2 + segmentLength;
                if (segmentLength < 2 || segmentEnd > jpeg.Length)
                    return false;

                int payloadOffset = offset + 4;
                int payloadLength = segmentLength - 2;
                if (marker == 0xEF && HasMagic(jpeg, payloadOffset, payloadLength))
                {
                    int jsonOffset = payloadOffset + Magic.Length;
                    int jsonLength = payloadLength - Magic.Length;
                    try
                    {
                        string json = Encoding.UTF8.GetString(jpeg, jsonOffset, jsonLength);
                        metadata = JsonUtility.FromJson<TeleimagerFrameMetadata>(json);
                        return metadata != null &&
                               metadata.protocol == "teleimager-jpeg-v2" &&
                               metadata.capture_timestamp_ms > 0;
                    }
                    catch (Exception)
                    {
                        metadata = null;
                        return false;
                    }
                }

                offset = segmentEnd;
            }
            return false;
        }

        private static bool HasMagic(byte[] data, int offset, int length)
        {
            if (length < Magic.Length)
                return false;
            for (int i = 0; i < Magic.Length; ++i)
            {
                if (data[offset + i] != Magic[i])
                    return false;
            }
            return true;
        }
    }
}
