/**
 * Copyright 2019-2020, Robotec.ai sp z o.o.
 * Authors: adam.dabrowski@robotec.ai, piotr.jaroszek@robotec.ai
 */

using UnityEngine;
using System.Threading;

namespace Ros2Native
{
    /// <summary>
    /// The principal class for handling ROS2 node and spinning thread.
    /// </summary>
    public class ROS2Unity
    {
        public ROS2Node node;
        private bool initialized = false;
        private bool spinning = false;
        private readonly object spinningLock = new object();

        Thread publishThread;
        public bool Ok()
        {
            return (node != null && node.Ok());
        }

        public ROS2Unity()
        {
            node = new ROS2Node();
        }

        public void Destroy() {
            if(node != null)
            {
                node.DestroyNode();
            }
        }

        void Tick()
        {
            lock (spinningLock) {
                spinning = true;
            }
            while (spinning)
            {

                if (node.Ok())
                {
                    node.SpinOnce();
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
