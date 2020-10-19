/**
 * Copyright 2019-2020, Robotec.ai sp z o.o.
 * Authors: piotr.jaroszek@robotec.ai, adam.dabrowski@robotec.ai
 */

using System;
using System.Collections.Generic;
using System.Linq;
using Simulator.Bridge.Data;
using UnityEngine;

namespace Simulator.Bridge
{
    [BridgeName("Ros2NativeBridge")]
    public class Ros2NativeBridgeFactory : IBridgeFactory
    {
        public IBridgeInstance CreateInstance() => new Ros2NativeBridgeInstance();

        public void Register(IBridgePlugin plugin)
        {
            Debug.Log("Register native bridge");

            // point cloud is special, as we use special writer for performance reasons
            plugin.AddType<PointCloudData>(typeof(PointCloudData).Name);
            plugin.AddPublisherCreator(
                (instance, topic) =>
                {
                    var ros2Instance = instance as Ros2NativeBridgeInstance;
                    ros2Instance.AddPublisher<sensor_msgs.msg.PointCloud2>(topic);
                    var writer = new Ros2NativePointCloudWriter(ros2Instance, topic);
                    return new Publisher<PointCloudData>((data, completed) => writer.Write(data, completed));
                }
            );

            RegPublisher<ImageData, sensor_msgs.msg.CompressedImage>(plugin, Ros2NativeConversions.ConvertFrom);
            RegPublisher<CanBusData, lgsvl_msgs.msg.CanBusData>(plugin, Ros2NativeConversions.ConvertFrom);
            RegPublisher<GpsData, sensor_msgs.msg.NavSatFix>(plugin, Ros2NativeConversions.ConvertFrom);
            RegPublisher<GpsOdometryData, nav_msgs.msg.Odometry>(plugin, Ros2NativeConversions.ConvertFrom);
            RegPublisher<ImuData, sensor_msgs.msg.Imu>(plugin, Ros2NativeConversions.ConvertFrom);
            RegPublisher<ClockData, rosgraph_msgs.msg.Clock>(plugin, Ros2NativeConversions.ConvertFrom);
            RegPublisher<VehicleOdometryData, lgsvl_msgs.msg.VehicleOdometry>(plugin, Ros2NativeConversions.ConvertFrom);

            RegSubscriber<VehicleStateData, lgsvl_msgs.msg.VehicleStateData>(plugin, Ros2NativeConversions.ConvertTo);
            RegSubscriber<VehicleControlData, lgsvl_msgs.msg.VehicleControlData>(plugin, Ros2NativeConversions.ConvertTo);
        }

        public void RegPublisher<DataType, BridgeType>(IBridgePlugin plugin, Func<DataType, BridgeType> converter)
        {
            plugin.AddType<DataType>(typeof(DataType).Name);
            plugin.AddPublisherCreator(
                (instance, topic) =>
                {
                    var ros2Instance = instance as Ros2NativeBridgeInstance;
                    ros2Instance.AddPublisher<BridgeType>(topic);
                    var writer = new Ros2NativeWriter<BridgeType>(ros2Instance, topic);
                    return new Publisher<DataType>((data, completed) => writer.Write(converter(data), completed));
                }
            );
        }

        public void RegSubscriber<DataType, BridgeType>(IBridgePlugin plugin, Func<BridgeType, DataType> converter)
        {
            plugin.AddType<DataType>(typeof(DataType).Name);
            plugin.AddSubscriberCreator<DataType>(
                (instance, topic, callback) => (instance as Ros2NativeBridgeInstance).AddSubscriber<BridgeType>(topic,
                    (data) => callback(converter(data))
                )
            );
        }
    }
}
