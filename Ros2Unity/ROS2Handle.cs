/**
 * Copyright 2019-2020, Robotec.ai sp z o.o.
 * Authors: adam.dabrowski@robotec.ai, piotr.jaroszek@robotec.ai
 */

using UnityEngine;
using System.Threading;
using ROS2;

namespace ROS2
{
    /// <summary>
    /// The principal class for handling ROS2 node and spinning thread.
    /// </summary>
    public class ROS2Handle
    {
        private ROS2ForUnity ros2forUnity;
        public ROS2Node node;
        private bool initialized = false;
        private bool spinning = false;
        private readonly object spinningLock = new object();

        Thread publishThread;
        public bool Ok()
        {
            return (node != null && ros2forUnity.Ok());
        }

        public ROS2Handle()
        {
            ros2forUnity = new ROS2ForUnity();
            node = new ROS2Node();
        }

        public void Destroy() {
            ros2forUnity.DestroyROS2ForUnity();
        }

        void Tick()
        {
            lock (spinningLock) {
                spinning = true;
            }
            while (spinning)
            {

                if (ros2forUnity.Ok())
                {
                    Ros2cs.SpinOnce(node.node, 0.01);
                }
            }
        }

        public void Stop()
        {
            lock (spinningLock) {
                spinning = false;
            }
            publishThread.Join();
        }

        public void Run()
        {
            if (!initialized)
            {
                publishThread = new Thread(() => Tick());
                publishThread.Start();
                initialized = true;
            }
        }
    }

}
