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
    [BridgeName("ROS2ForUnitySVLBridge", "ROS2")]
    public class ROS2ForUnitySVLBridgeFactory : IBridgeFactory
    {
        public IBridgeInstance CreateInstance() => new ROS2ForUnitySVLBridgeInstance();

        public void Register(IBridgePlugin plugin)
        {
            Debug.Log("Register native bridge!!");

            // point cloud is special, as we use special writer for performance reasons
            plugin.AddType<PointCloudData>(typeof(PointCloudData).Name);
            plugin.AddPublisherCreator(
                (instance, topic) =>
                {
                    var ros2Instance = instance as ROS2ForUnitySVLBridgeInstance;
                    ros2Instance.AddPublisher<sensor_msgs.msg.PointCloud2>(topic);
                    var writer = new ROS2ForUnitySVLBridgePointCloudWriter(ros2Instance, topic);
                    return new Publisher<PointCloudData>((data, completed) => writer.Write(data, completed));
                }
            );

            RegPublisher<ImageData, sensor_msgs.msg.CompressedImage>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<CameraInfoData, sensor_msgs.msg.CameraInfo>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<Detected3DObjectData, lgsvl_msgs.msg.Detection3DArray>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<Detected2DObjectData, lgsvl_msgs.msg.Detection2DArray>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<CanBusData, lgsvl_msgs.msg.CanBusData>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<GpsData, sensor_msgs.msg.NavSatFix>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<GpsOdometryData, nav_msgs.msg.Odometry>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<ImuData, sensor_msgs.msg.Imu>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<ClockData, rosgraph_msgs.msg.Clock>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);
            RegPublisher<VehicleOdometryData, lgsvl_msgs.msg.VehicleOdometry>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertFrom);

            RegSubscriber<VehicleStateData, lgsvl_msgs.msg.VehicleStateData>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertTo);
            RegSubscriber<VehicleControlData, lgsvl_msgs.msg.VehicleControlData>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertTo);
            RegSubscriber<Detected2DObjectArray, lgsvl_msgs.msg.Detection2DArray>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertTo);
            RegSubscriber<Detected3DObjectArray, lgsvl_msgs.msg.Detection3DArray>(plugin, ROS2ForUnitySVLBridgeConversions.ConvertTo);
        }

        public void RegPublisher<DataType, BridgeType>(IBridgePlugin plugin, Func<DataType, BridgeType> converter)
        {
            plugin.AddType<DataType>(typeof(DataType).Name);
            plugin.AddPublisherCreator(
                (instance, topic) =>
                {
                    var ros2Instance = instance as ROS2ForUnitySVLBridgeInstance;
                    ros2Instance.AddPublisher<BridgeType>(topic);
                    var writer = new ROS2ForUnitySVLBridgeWriter<BridgeType>(ros2Instance, topic);
                    return new Publisher<DataType>((data, completed) => writer.Write(converter(data), completed));
                }
            );
        }

        public void RegSubscriber<DataType, BridgeType>(IBridgePlugin plugin, Func<BridgeType, DataType> converter)
        {
            plugin.AddType<DataType>(typeof(DataType).Name);
            plugin.AddSubscriberCreator<DataType>(
                (instance, topic, callback) => (instance as ROS2ForUnitySVLBridgeInstance).AddSubscriber<BridgeType>(topic,
                    (data) => callback(converter(data))
                )
            );
        }
    }
}
