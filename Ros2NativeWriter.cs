/**
 * Copyright 2019-2020, Robotec.ai sp z o.o.
 * Authors: piotr.jaroszek@robotec.ai, adam.dabrowski@robotec.ai
 */

using System;
using System.Text;

namespace Simulator.Bridge
{
    public class Ros2NativeWriter<BridgeType>
    {
        Ros2NativeInstance Instance;
        string Topic;

        public Ros2NativeWriter(Ros2NativeInstance instance, string topic)
        {
            Instance = instance;
            Topic = topic;
        }

        public void Write(BridgeType message, Action completed)
        {
            Instance.Publish<BridgeType>(Topic, message);
            if(completed != null) completed();
        }
    }

    public class Ros2NativePointCloudWriter
    {
        Ros2NativeWriter<sensor_msgs.msg.PointCloud2> Writer;

        byte[] Buffer;

        public Ros2NativePointCloudWriter(Ros2NativeInstance instance, string topic)
        {
            Writer = new Ros2NativeWriter<sensor_msgs.msg.PointCloud2>(instance, topic);
        }

        public void Write(Data.PointCloudData data, Action completed)
        {
            if (Buffer == null || Buffer.Length != data.Points.Length)
            {
                Buffer = new byte[32 * data.Points.Length];
            }

            int count = 0;
            unsafe
            {
                fixed (byte* ptr = Buffer)
                {
                    int offset = 0;
                    for (int i = 0; i < data.Points.Length; i++)
                    {
                        var point = data.Points[i];
                        if (point == UnityEngine.Vector4.zero)
                        {
                            continue;
                        }

                        var pos = new UnityEngine.Vector3(point.x, point.y, point.z);
                        float intensity = point.w;

                        *(UnityEngine.Vector3*)(ptr + offset) = data.Transform.MultiplyPoint3x4(pos);
                        *(ptr + offset + 16) = (byte)(intensity * 255);

                        offset += 32;
                        count++;
                    }
                }
            }

            var time = Ros2NativeConversions.ConvertTime(data.Time);
            var msg = new sensor_msgs.msg.PointCloud2()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Height = 1,
                Width = (uint)count,
                Fields = new []
                {
                    new sensor_msgs.msg.PointField()
                    {
                        Name = "x",
                        Offset = 0,
                        Datatype = 7,
                        Count = 1,
                    },
                    new sensor_msgs.msg.PointField()
                    {
                        Name = "y",
                        Offset = 4,
                        Datatype = 7,
                        Count = 1,
                    },
                    new sensor_msgs.msg.PointField()
                    {
                        Name = "z",
                        Offset = 8,
                        Datatype = 7,
                        Count = 1,
                    },
                    new sensor_msgs.msg.PointField()
                    {
                        Name = "intensity",
                        Offset = 16,
                        Datatype = 2,
                        Count = 1,
                    },
                    new sensor_msgs.msg.PointField()
                    {
                        Name = "timestamp",
                        Offset = 24,
                        Datatype = 8,
                        Count = 1,
                    },
                },
                Is_bigendian = false,
                Point_step = 32,
                Row_step = (uint)count * 32,
                Is_dense = true
            };
            msg.Data = new byte[count * 32];
            System.Buffer.BlockCopy(Buffer, 0, msg.Data, 0, count * 32);

            Writer.Write(msg, completed);
        }
    }
}
