/**
 * Copyright 2019-2020, Robotec.ai sp z o.o.
 * Authors: piotr.jaroszek@robotec.ai, adam.dabrowski@robotec.ai 
 */

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
    public partial class Ros2NativeBridgeInstance : IBridgeInstance
    {
        public Status Status { get; private set; } = Status.Disconnected;

        Dictionary<string, dynamic> Publishers = new Dictionary<string, dynamic>();
        Dictionary<string, dynamic> Subscribers = new Dictionary<string, dynamic>();

        private ROS2Handle Ros2Handler;

        public Ros2NativeBridgeInstance()
        {
            Ros2Handler = new ROS2Handle();
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

            if (type == typeof(lgsvl_msgs.msg.VehicleControlData))
            {
                var subscriber = Ros2Handler.node.CreateSubscription<lgsvl_msgs.msg.VehicleControlData>
                (
                    topic,
                    (callback as Action<lgsvl_msgs.msg.VehicleControlData>)
                );
                Subscribers.Add(topic, subscriber);
            } else if (type == typeof(lgsvl_msgs.msg.VehicleStateData))
            {
                var subscriber = Ros2Handler.node.CreateSubscription<lgsvl_msgs.msg.VehicleStateData>
                (
                    topic,
                    (callback as Action<lgsvl_msgs.msg.VehicleStateData>)
                );
                Subscribers.Add(topic, subscriber);
            } else if (type == typeof(lgsvl_msgs.msg.Detection3DArray))
            {
                var subscriber = Ros2Handler.node.CreateSubscription<lgsvl_msgs.msg.Detection3DArray>
                (
                    topic,
                    (callback as Action<lgsvl_msgs.msg.Detection3DArray>)
                );
                Subscribers.Add(topic, subscriber);
            } else if (type == typeof(lgsvl_msgs.msg.Detection2DArray))
            {
                var subscriber = Ros2Handler.node.CreateSubscription<lgsvl_msgs.msg.Detection2DArray>
                (
                    topic,
                    (callback as Action<lgsvl_msgs.msg.Detection2DArray>)
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
                var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.Imu>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(nav_msgs.msg.Odometry))
            {
                var publisher = Ros2Handler.node.CreatePublisher<nav_msgs.msg.Odometry>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(sensor_msgs.msg.PointCloud2))
            {
                var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.PointCloud2>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(sensor_msgs.msg.CompressedImage))
            {
                var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.CompressedImage>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(lgsvl_msgs.msg.CanBusData))
            {
                var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.CanBusData>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(sensor_msgs.msg.NavSatFix))
            {
                var publisher = Ros2Handler.node.CreatePublisher<sensor_msgs.msg.NavSatFix>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(rosgraph_msgs.msg.Clock))
            {
                var publisher = Ros2Handler.node.CreatePublisher<rosgraph_msgs.msg.Clock>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(lgsvl_msgs.msg.VehicleOdometry))
            {
                var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.VehicleOdometry>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(lgsvl_msgs.msg.Detection3DArray))
            {
                var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.Detection3DArray>(topic);
                Publishers.Add(topic, publisher);
            } else if (type == typeof(lgsvl_msgs.msg.Detection2DArray))
            {
                var publisher = Ros2Handler.node.CreatePublisher<lgsvl_msgs.msg.Detection2DArray>(topic);
                Publishers.Add(topic, publisher);
            }
            return;
        }

        public void Publish<BridgeType>(string topic, BridgeType msg)
        {
            if (Ros2Handler.Ok()) {
                Publishers[topic].Publish(msg);
            }
        }
    }
}
