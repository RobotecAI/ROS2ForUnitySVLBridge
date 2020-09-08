/**
 * Copyright (c) 2020 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using System;
using System.Text;
using System.Linq;
using System.Net.Sockets;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using Ros2Native;

namespace Simulator.Bridge
{
    public partial class Ros2NativeBridgeInstance : IBridgeInstance
    {
        public Status Status { get; private set; } = Status.Disconnected;

        Dictionary<string, dynamic> Publishers = new Dictionary<string, dynamic>();
        Dictionary<string, dynamic> Subscribers = new Dictionary<string, dynamic>();

        private ROS2Unity ros2UnityNode;
        
        public Ros2NativeBridgeInstance()
        {
            Debug.Log("Instance of native bridge");

            Ros2Native.ROS2Node.CheckROSRMWSourced();
            Ros2Native.ROS2Node.CheckROSVersionSourced();

            ROS2.Ros2cs.Init();
            ros2UnityNode = new ROS2Unity();
            Debug.Log("Using " + ROS2.Ros2cs.GetRMWImplementationID() + " rmw.");
        }

        void WaitForRos2()
        {
            Debug.Log("Wait for ros2");
            while (!ros2UnityNode.Ok())
            {
                Thread.Sleep(200);
            };
            Debug.Log("Wait for ros2 ended");
        }

        public void Connect(string connection)
        {
            Debug.Log("Native connect");
            ros2UnityNode.Run();

            WaitForRos2();
            Status = Status.Connected; 
            Debug.Log("Native connected");
        }

        public void Disconnect()
        {
            ros2UnityNode.Stop();

            if(ros2UnityNode.Ok()) {
                ros2UnityNode.Destroy();
            }
            Status = Status.Disconnected;
        }

        public void AddSubscriber<BridgeType>(string topic, Action<BridgeType> callback)
        {
            WaitForRos2();
            var type = typeof(BridgeType);

            if (type == typeof(lgsvl_msgs.msg.VehicleControlData))
            {
                var subscriber = ros2UnityNode.node.CreateSubscription<lgsvl_msgs.msg.VehicleControlData>
                (
                    topic,
                    (callback as Action<lgsvl_msgs.msg.VehicleControlData>)
                );
                Subscribers.Add(topic, subscriber);
            } else if (type == typeof(lgsvl_msgs.msg.VehicleStateData))
            {
                var subscriber = ros2UnityNode.node.CreateSubscription<lgsvl_msgs.msg.VehicleStateData>
                (
                    topic,
                    (callback as Action<lgsvl_msgs.msg.VehicleStateData>)
                );
                Subscribers.Add(topic, subscriber);
            }
        }

        public void AddPublisher<BridgeType>(string topic)
        {
            WaitForRos2();
            var type = typeof(BridgeType);

            if (type == typeof(sensor_msgs.msg.Imu))
            {
                var publisher = ros2UnityNode.node.CreateSensorPublisher<sensor_msgs.msg.Imu>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(nav_msgs.msg.Odometry))
            {
                var publisher = ros2UnityNode.node.CreateSensorPublisher<nav_msgs.msg.Odometry>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(sensor_msgs.msg.PointCloud2))
            {
                var publisher = ros2UnityNode.node.CreateSensorPublisher<sensor_msgs.msg.PointCloud2>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(sensor_msgs.msg.CompressedImage))
            {
                var publisher = ros2UnityNode.node.CreateSensorPublisher<sensor_msgs.msg.CompressedImage>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(lgsvl_msgs.msg.CanBusData))
            {
                var publisher = ros2UnityNode.node.CreatePublisher<lgsvl_msgs.msg.CanBusData>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(sensor_msgs.msg.NavSatFix))
            {
                var publisher = ros2UnityNode.node.CreateSensorPublisher<sensor_msgs.msg.NavSatFix>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(rosgraph_msgs.msg.Clock))
            {
                var publisher = ros2UnityNode.node.CreatePublisher<rosgraph_msgs.msg.Clock>(topic);
                Publishers.Add(topic, publisher);
            }
            return;
        }



        public void Publish<BridgeType>(string topic, BridgeType msg)
        {
            if (ros2UnityNode.Ok()) {
                Publishers[topic].Publish(msg);
            }
        }
    }
}
