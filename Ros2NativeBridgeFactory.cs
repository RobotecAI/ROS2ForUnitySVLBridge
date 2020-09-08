/**
 * Copyright (c) 2019 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
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

            // RegPublisher<ImageData, Ros.CompressedImage>(plugin, Ros2Conversions.ConvertFrom);
            // RegPublisher<Detected3DObjectData, Lgsvl.Detection3DArray>(plugin, Ros2Conversions.ConvertFrom);
            // RegPublisher<Detected2DObjectData, Lgsvl.Detection2DArray>(plugin, Ros2Conversions.ConvertFrom);
            // RegPublisher<SignalDataArray, Lgsvl.SignalArray>(plugin, Ros2Conversions.ConvertFrom);
            // RegPublisher<CanBusData, Lgsvl.CanBusData>(plugin, Ros2Conversions.ConvertFrom);
            // RegPublisher<UltrasonicData, Lgsvl.Ultrasonic>(plugin, Ros2Conversions.ConvertFrom);
            RegPublisher<GpsData, sensor_msgs.msg.NavSatFix>(plugin, Ros2NativeConversions.ConvertFrom);
            // RegPublisher<GpsOdometryData, Ros.Odometry>(plugin, Ros2Conversions.ConvertFrom);
            // RegPublisher<ImuData, Ros.Imu>(plugin, Ros2Conversions.ConvertFrom);
            // RegPublisher<ClockData, Ros.Clock>(plugin, Ros2Conversions.ConvertFrom);

            // RegSubscriber<VehicleStateData, Lgsvl.VehicleStateData>(plugin, Ros2Conversions.ConvertTo);
            // RegSubscriber<VehicleControlData, Lgsvl.VehicleControlData>(plugin, Ros2Conversions.ConvertTo);
            // RegSubscriber<Detected2DObjectArray, Lgsvl.Detection2DArray>(plugin, Ros2Conversions.ConvertTo);
            // RegSubscriber<Detected3DObjectArray, Lgsvl.Detection3DArray>(plugin, Ros2Conversions.ConvertTo);
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
            // plugin.AddType<DataType>(Ros2Utils.GetMessageType<BridgeType>());
            // plugin.AddSubscriberCreator<DataType>(
            //     (instance, topic, callback) => (instance as Ros2BridgeInstance).AddSubscriber<BridgeType>(topic,
            //         rawData => callback(converter(Ros2Serialization.Unserialize<BridgeType>(rawData)))
            //     )
            // );
        }
    }
}
