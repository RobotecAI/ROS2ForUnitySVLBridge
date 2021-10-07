// Copyright 2019-2021 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Text;
using UnityEngine;

namespace Simulator.Bridge
{
    public class ROS2ForUnitySVLBridgeWriter<BridgeType>
    {
        ROS2ForUnitySVLBridgeInstance Instance;
        string Topic;

        public ROS2ForUnitySVLBridgeWriter(ROS2ForUnitySVLBridgeInstance instance, string topic)
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

    public class ROS2ForUnitySVLBridgeCompressedImageWriter
    {
        ROS2ForUnitySVLBridgeWriter<sensor_msgs.msg.CompressedImage> Writer;
        sensor_msgs.msg.CompressedImage msg;

        public ROS2ForUnitySVLBridgeCompressedImageWriter(ROS2ForUnitySVLBridgeInstance instance, string topic)
        {
            Writer = new ROS2ForUnitySVLBridgeWriter<sensor_msgs.msg.CompressedImage>(instance, topic);
        }

        public void Write(Data.ImageData data, Action completed)
        {
            if (msg == null || msg.Data.Length != data.Length)
            {
                var time = ROS2ForUnitySVLBridgeConversions.ConvertTime(data.Time);
                msg = new sensor_msgs.msg.CompressedImage() {
                    Header = new std_msgs.msg.Header()
                    {
                        Stamp = time,
                        Frame_id = data.Frame,
                    },
                    Format = "jpeg"
                };
                msg.Data = new byte[data.Length];
            }
            
            System.Buffer.BlockCopy(data.Bytes, 0, msg.Data, 0, data.Length);
            Writer.Write(msg, completed);
        }
    }

    public class ROS2ForUnitySVLBridgePointCloudWriter
    {
        ROS2ForUnitySVLBridgeWriter<sensor_msgs.msg.PointCloud2> Writer;

        byte[] Buffer;

        public ROS2ForUnitySVLBridgePointCloudWriter(ROS2ForUnitySVLBridgeInstance instance, string topic)
        {
            Writer = new ROS2ForUnitySVLBridgeWriter<sensor_msgs.msg.PointCloud2>(instance, topic);
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

            var time = ROS2ForUnitySVLBridgeConversions.ConvertTime(data.Time);
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
