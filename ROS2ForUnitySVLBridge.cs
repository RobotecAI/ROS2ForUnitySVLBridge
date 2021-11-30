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
using System.Linq;
using System.Net.Sockets;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using ROS2;

namespace Simulator.Bridge
{
    using ImuDict = Dictionary<string, ROS2.Publisher<sensor_msgs.msg.Imu>>;
    using OdometryDict = Dictionary<string, ROS2.Publisher<nav_msgs.msg.Odometry>>;
    using PointCloud2Dict = Dictionary<string, ROS2.Publisher<sensor_msgs.msg.PointCloud2>>;
    using CompressedImageDict = Dictionary<string, ROS2.Publisher<sensor_msgs.msg.CompressedImage>>;
    using CanBusDataDict = Dictionary<string, ROS2.Publisher<lgsvl_msgs.msg.CanBusData>>;
    using NavSatFixDict = Dictionary<string, ROS2.Publisher<sensor_msgs.msg.NavSatFix>>;
    using ClockDict = Dictionary<string, ROS2.Publisher<rosgraph_msgs.msg.Clock>>;
    using VehicleOdometryDict = Dictionary<string, ROS2.Publisher<lgsvl_msgs.msg.VehicleOdometry>>;
    using Detection3DArrayDict = Dictionary<string, ROS2.Publisher<lgsvl_msgs.msg.Detection3DArray>>;
    using Detection2DArrayDict = Dictionary<string, ROS2.Publisher<lgsvl_msgs.msg.Detection2DArray>>;
    using CameraInfoDict = Dictionary<string, ROS2.Publisher<sensor_msgs.msg.CameraInfo>>;
    using LaserScanDict = Dictionary<string, ROS2.Publisher<sensor_msgs.msg.LaserScan>>;
    using DetectedRadarObjectsArrayDict = Dictionary<string, ROS2.Publisher<lgsvl_msgs.msg.DetectedRadarObjectArray>>;

    public class ROS2Publishers {

        public ImuDict ImuPublishers = new ImuDict();
        public OdometryDict OdometryPublishers = new OdometryDict();
        public PointCloud2Dict PointCloud2Publishers = new PointCloud2Dict();
        public CompressedImageDict CompressedImagePublishers = new CompressedImageDict();
        public CanBusDataDict CanBusDataPublishers = new CanBusDataDict();
        public NavSatFixDict NavSatFixPublishers = new NavSatFixDict();
        public ClockDict ClockPublishers = new ClockDict();
        public VehicleOdometryDict VehicleOdometryPublishers = new VehicleOdometryDict();
        public Detection3DArrayDict Detection3DArrayPublishers = new Detection3DArrayDict();
        public Detection2DArrayDict Detection2DArrayPublishers = new Detection2DArrayDict();
        public CameraInfoDict CameraInfoPublishers = new CameraInfoDict();
        public LaserScanDict LaserScanPublishers = new LaserScanDict();
        public DetectedRadarObjectsArrayDict DetectedRadarObjectsArrayPublishers = new DetectedRadarObjectsArrayDict();
    }

    public partial class ROS2ForUnitySVLBridgeInstance : IBridgeInstance
    {

        public Status Status { get; private set; } = Status.Disconnected;

        
        ROS2Publishers Publishers;
        
        private ROS2Handle Ros2Handler;

        public ROS2ForUnitySVLBridgeInstance()
        {
            Ros2Handler = new ROS2Handle();
            Publishers = new ROS2Publishers();
        }

        void WaitForRos2()
        {
            while (!Ros2Handler.Ok())
            {
                Thread.Sleep(200);
            };
        }

        public void Connect(string connection)
        {
            Ros2Handler.Run();

            WaitForRos2();
            Status = Status.Connected;
        }

        public void Disconnect()
        {
            Ros2Handler.Stop();

            if(Ros2Handler.Ok()) {
                Ros2Handler.Destroy();
            }
            Status = Status.Disconnected;
        }

        public void AddSubscriber<BridgeType>(string topic, Action<BridgeType> callback)
        {
            WaitForRos2();
            var type = typeof(BridgeType);
            try
            {
                if (type == typeof(lgsvl_msgs.msg.VehicleControlData))
                {
                    var subscriber = Ros2Handler.node.CreateSubscription<lgsvl_msgs.msg.VehicleControlData>
                    (
                        topic,
                        (callback as Action<lgsvl_msgs.msg.VehicleControlData>)
                    );
                    // Subscribers.Add(topic, subscriber);
                } else if (type == typeof(lgsvl_msgs.msg.VehicleStateData))
                {
                    var subscriber = Ros2Handler.node.CreateSubscription<lgsvl_msgs.msg.VehicleStateData>
                    (
                        topic,
                        (callback as Action<lgsvl_msgs.msg.VehicleStateData>)
                    );
                    // Subscribers.Add(topic, subscriber);
                } else if (type == typeof(lgsvl_msgs.msg.Detection3DArray))
                {
                    var subscriber = Ros2Handler.node.CreateSubscription<lgsvl_msgs.msg.Detection3DArray>
                    (
                        topic,
                        (callback as Action<lgsvl_msgs.msg.Detection3DArray>)
                    );
                    // Subscribers.Add(topic, subscriber);
                } else if (type == typeof(lgsvl_msgs.msg.Detection2DArray))
                {
                    var subscriber = Ros2Handler.node.CreateSubscription<lgsvl_msgs.msg.Detection2DArray>
                    (
                        topic,
                        (callback as Action<lgsvl_msgs.msg.Detection2DArray>)
                    );
                    // Subscribers.Add(topic, subscriber);
                }
            } catch (InvalidOperationException e)
            {
                Debug.LogWarning(e.Message);
            }
        }

        public void AddPublisher<BridgeType>(string topic)
        {
            WaitForRos2();
            var type = typeof(BridgeType);
            try
            {
                if (type == typeof(sensor_msgs.msg.Imu))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.Imu>(topic);
                    Publishers.ImuPublishers.Add(topic, publisher);
                } else if (type == typeof(nav_msgs.msg.Odometry))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<nav_msgs.msg.Odometry>(topic);
                    Publishers.OdometryPublishers.Add(topic, publisher);
                } else if (type == typeof(sensor_msgs.msg.PointCloud2))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.PointCloud2>(topic);
                    Publishers.PointCloud2Publishers.Add(topic, publisher);
                } else if (type == typeof(sensor_msgs.msg.CompressedImage))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.CompressedImage>(topic);
                    Publishers.CompressedImagePublishers.Add(topic, publisher);
                } else if (type == typeof(lgsvl_msgs.msg.CanBusData))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.CanBusData>(topic);
                    Publishers.CanBusDataPublishers.Add(topic, publisher);
                } else if (type == typeof(sensor_msgs.msg.NavSatFix))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.NavSatFix>(topic);
                    Publishers.NavSatFixPublishers.Add(topic, publisher);
                } else if (type == typeof(rosgraph_msgs.msg.Clock))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<rosgraph_msgs.msg.Clock>(topic);
                    Publishers.ClockPublishers.Add(topic, publisher);
                } else if (type == typeof(lgsvl_msgs.msg.VehicleOdometry))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.VehicleOdometry>(topic);
                    Publishers.VehicleOdometryPublishers.Add(topic, publisher);
                } else if (type == typeof(lgsvl_msgs.msg.Detection3DArray))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.Detection3DArray>(topic);
                    Publishers.Detection3DArrayPublishers.Add(topic, publisher);
                } else if (type == typeof(lgsvl_msgs.msg.Detection2DArray))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.Detection2DArray>(topic);
                    Publishers.Detection2DArrayPublishers.Add(topic, publisher);
                } else if (type == typeof(sensor_msgs.msg.CameraInfo))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.CameraInfo>(topic);
                    Publishers.CameraInfoPublishers.Add(topic, publisher);
                } else if (type == typeof(sensor_msgs.msg.LaserScan))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.LaserScan>(topic);
                    Publishers.LaserScanPublishers.Add(topic, publisher);
                } else if (type == typeof(lgsvl_msgs.msg.DetectedRadarObjectArray))
                {
                    var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.DetectedRadarObjectArray>(topic);
                    Publishers.DetectedRadarObjectsArrayPublishers.Add(topic, publisher);
                }
                return;
            } catch (InvalidOperationException e)
            {
                Debug.LogWarning(e.Message);
            }
        }

        public void Publish<BridgeType>(string topic, BridgeType msg)
        {
            if (Ros2Handler.Ok()) {
                if(typeof(BridgeType) == typeof(sensor_msgs.msg.Imu)) {Publishers.ImuPublishers[topic].Publish(msg as sensor_msgs.msg.Imu);}
                else if(typeof(BridgeType) == typeof(nav_msgs.msg.Odometry)) {Publishers.OdometryPublishers[topic].Publish(msg as nav_msgs.msg.Odometry);}
                else if(typeof(BridgeType) == typeof(sensor_msgs.msg.PointCloud2)) {Publishers.PointCloud2Publishers[topic].Publish(msg as sensor_msgs.msg.PointCloud2);}
                else if(typeof(BridgeType) == typeof(sensor_msgs.msg.CompressedImage)) {Publishers.CompressedImagePublishers[topic].Publish(msg as sensor_msgs.msg.CompressedImage);}
                else if(typeof(BridgeType) == typeof(lgsvl_msgs.msg.CanBusData)) {Publishers.CanBusDataPublishers[topic].Publish(msg as lgsvl_msgs.msg.CanBusData);}
                else if(typeof(BridgeType) == typeof(sensor_msgs.msg.NavSatFix)) {Publishers.NavSatFixPublishers[topic].Publish(msg as sensor_msgs.msg.NavSatFix);}
                else if(typeof(BridgeType) == typeof(rosgraph_msgs.msg.Clock)) {Publishers.ClockPublishers[topic].Publish(msg as rosgraph_msgs.msg.Clock);}
                else if(typeof(BridgeType) == typeof(lgsvl_msgs.msg.VehicleOdometry)) {Publishers.VehicleOdometryPublishers[topic].Publish(msg as lgsvl_msgs.msg.VehicleOdometry);}
                else if(typeof(BridgeType) == typeof(lgsvl_msgs.msg.Detection3DArray)) {Publishers.Detection3DArrayPublishers[topic].Publish(msg as lgsvl_msgs.msg.Detection3DArray);}
                else if(typeof(BridgeType) == typeof(lgsvl_msgs.msg.Detection2DArray)) {Publishers.Detection2DArrayPublishers[topic].Publish(msg as lgsvl_msgs.msg.Detection2DArray);}
                else if(typeof(BridgeType) == typeof(sensor_msgs.msg.CameraInfo)) {Publishers.CameraInfoPublishers[topic].Publish(msg as sensor_msgs.msg.CameraInfo);}
                else if(typeof(BridgeType) == typeof(sensor_msgs.msg.LaserScan)) {Publishers.LaserScanPublishers[topic].Publish(msg as sensor_msgs.msg.LaserScan);}
                else if(typeof(BridgeType) == typeof(lgsvl_msgs.msg.DetectedRadarObjectArray)) {Publishers.DetectedRadarObjectsArrayPublishers[topic].Publish(msg as lgsvl_msgs.msg.DetectedRadarObjectArray);}
            }
        }
    }
}
