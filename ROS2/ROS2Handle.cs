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
        private ROS2Unity ros2Unity;
        public ROS2Node node;
        private bool initialized = false;
        private bool spinning = false;
        private readonly object spinningLock = new object();

        Thread publishThread;
        public bool Ok()
        {
            return (node != null && ros2Unity.Ok());
        }

        public ROS2Handle()
        {
            ros2Unity = new ROS2Unity();
            node = new ROS2Node();
        }

        public void Destroy() {
            ros2Unity.DestroyROS2Unity();
        }

        void Tick()
        {
            lock (spinningLock) {
                spinning = true;
            }
            while (spinning)
            {

                if (ros2Unity.Ok())
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
