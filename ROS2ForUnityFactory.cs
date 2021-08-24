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
using System.Collections.Generic;
using System.Linq;
using Simulator.Bridge.Data;
using UnityEngine;

namespace Simulator.Bridge
{
    [BridgeName("ROS2ForUnity", "ROS2")]
    public class ROS2ForUnityFactory : IBridgeFactory
    {
        public IBridgeInstance CreateInstance() => new ROS2ForUnityInstance();

        public void Register(IBridgePlugin plugin)
        {
            Debug.Log("Register native bridge");

            // point cloud is special, as we use special writer for performance reasons
            plugin.AddType<PointCloudData>(typeof(PointCloudData).Name);
            plugin.AddPublisherCreator(
                (instance, topic) =>
                {
                    var ros2Instance = instance as ROS2ForUnityInstance;
                    ros2Instance.AddPublisher<sensor_msgs.msg.PointCloud2>(topic);
                    var writer = new ROS2ForUnityPointCloudWriter(ros2Instance, topic);
                    return new Publisher<PointCloudData>((data, completed) => writer.Write(data, completed));
                }
            );

            RegPublisher<ImageData, sensor_msgs.msg.CompressedImage>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<CameraInfoData, sensor_msgs.msg.CameraInfo>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<Detected3DObjectData, lgsvl_msgs.msg.Detection3DArray>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<Detected2DObjectData, lgsvl_msgs.msg.Detection2DArray>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<CanBusData, lgsvl_msgs.msg.CanBusData>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<GpsData, sensor_msgs.msg.NavSatFix>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<GpsOdometryData, nav_msgs.msg.Odometry>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<ImuData, sensor_msgs.msg.Imu>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<ClockData, rosgraph_msgs.msg.Clock>(plugin, ROS2ForUnityConversions.ConvertFrom);
            RegPublisher<VehicleOdometryData, lgsvl_msgs.msg.VehicleOdometry>(plugin, ROS2ForUnityConversions.ConvertFrom);

            RegSubscriber<VehicleStateData, lgsvl_msgs.msg.VehicleStateData>(plugin, ROS2ForUnityConversions.ConvertTo);
            RegSubscriber<VehicleControlData, lgsvl_msgs.msg.VehicleControlData>(plugin, ROS2ForUnityConversions.ConvertTo);
            RegSubscriber<Detected2DObjectArray, lgsvl_msgs.msg.Detection2DArray>(plugin, ROS2ForUnityConversions.ConvertTo);
            RegSubscriber<Detected3DObjectArray, lgsvl_msgs.msg.Detection3DArray>(plugin, ROS2ForUnityConversions.ConvertTo);
        }

        public void RegPublisher<DataType, BridgeType>(IBridgePlugin plugin, Func<DataType, BridgeType> converter)
        {
            plugin.AddType<DataType>(typeof(DataType).Name);
            plugin.AddPublisherCreator(
                (instance, topic) =>
                {
                    var ros2Instance = instance as ROS2ForUnityInstance;
                    ros2Instance.AddPublisher<BridgeType>(topic);
                    var writer = new ROS2ForUnityWriter<BridgeType>(ros2Instance, topic);
                    return new Publisher<DataType>((data, completed) => writer.Write(converter(data), completed));
                }
            );
        }

        public void RegSubscriber<DataType, BridgeType>(IBridgePlugin plugin, Func<BridgeType, DataType> converter)
        {
            plugin.AddType<DataType>(typeof(DataType).Name);
            plugin.AddSubscriberCreator<DataType>(
                (instance, topic, callback) => (instance as ROS2ForUnityInstance).AddSubscriber<BridgeType>(topic,
                    (data) => callback(converter(data))
                )
            );
        }
    }
}
