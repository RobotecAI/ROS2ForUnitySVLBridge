using UnityEngine;
using System.Threading;

namespace Ros2Native
{
    /// <summary>
    /// The principal MonoBehaviour class for handling ROS2Node. This is the component find and use in all
    /// other scripts that need to access ROS2 (publish something, receive messages, etc)
    /// </summary>
    public class ROS2Unity
    {
        public ROS2Node node;
        // public ROS2Clock clock;
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
            // Debug.Log("New ROS2 Node");
            node = new ROS2Node();
            // clock = new ROS2Clock(this);
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
                    // clock.Tick();
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