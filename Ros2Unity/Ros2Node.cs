/**
 * Copyright 2019-2020, Robotec.ai sp z o.o.
 */

using ROS2;
using System;
using UnityEngine;
using UnityEditor;

namespace Ros2Native
{
    /// <summary>
    /// A class responsible for handling all ROS2 node related actions, including checking for proper initialization,
    /// creating and managing context and node, creating publishers and subscriptions and a spin for callbacks.
    /// </summary>
    public class ROS2Node
    {
        private INode node;
        public static bool isInitialized = false;

        private double timeout = 0.001;

        public static void EnsureROS2PluginVisibility()
        {
#if UNITY_EDITOR
            string currentLDPath = Environment.GetEnvironmentVariable("LD_LIBRARY_PATH");

            string pluginPath = Application.dataPath + "/Plugins/x86_64";

            if (string.IsNullOrEmpty(currentLDPath) || !currentLDPath.Contains(pluginPath))
            {
                EditorApplication.isPlaying = false;
                throw new System.InvalidOperationException("Missing library path to plugins in environment. Make sure your 'LD_LIBRARY_PATH' environment variable points to 'Assets/Plugins/x86_64' directory of this project.");

                //NOTE: Setting LD_LIBRARY_PATH doesn't work at this stage unfortunately, even in static constructor.
            }
#endif
        }

        public static void CheckROSVersionSourced()
        {
            string currentVersion = Environment.GetEnvironmentVariable("ROS_DISTRO");
            const string supportedVersion = "dashing";
            if (string.IsNullOrEmpty(currentVersion))
            {
                string errMessage = "No ROS environment sourced. You need to source your ROS2 " + supportedVersion + " environment before launching simulator.";
                Debug.LogError(errMessage);
#if UNITY_EDITOR
                EditorApplication.isPlaying = false;
                throw new System.InvalidOperationException(errMessage);
#else
            throw new System.InvalidOperationException(errMessage);
#endif
            }

            if (currentVersion != supportedVersion)
            {
                string errMessage = "Currently sourced ROS version differs from supported one. Sourced: " + currentVersion + ", supported: " + supportedVersion;
                Debug.LogError(errMessage);
#if UNITY_EDITOR
                EditorApplication.isPlaying = false;
                throw new System.InvalidOperationException(errMessage);
#else
            const int ROS_BAD_VERSION_CODE = 34;
#endif
            }
        }

        public static void CheckROSRMWSourced()
        {
            string currentRMW = Environment.GetEnvironmentVariable("RMW_IMPLEMENTATION");
            const string supportedRMW = "rmw_cyclonedds_cpp";
            if (string.IsNullOrEmpty(currentRMW))
            {
                string errMessage = "No ROS RMW set. You need to set RMW_IMPLEMENTATION to " + supportedRMW;
                Debug.LogError(errMessage);
#if UNITY_EDITOR
                EditorApplication.isPlaying = false;
                throw new System.InvalidOperationException(errMessage);
#else
            throw new System.InvalidOperationException(errMessage);
#endif
            }

            if (currentRMW != supportedRMW)
            {
                string errMessage = "Currently ROS2 RMW differs from supported one. Sourced: " + currentRMW + ", supported: " + supportedRMW;
                Debug.LogError(errMessage);
#if UNITY_EDITOR
                EditorApplication.isPlaying = false;
                throw new System.InvalidOperationException(errMessage);
#else
            throw new System.InvalidOperationException(errMessage);
#endif
            }
        }

        private void CreateNode()
        {
            const string unityROS2NodeName = "UnityROS2Node";
            node = Ros2cs.CreateNode(unityROS2NodeName);
            isInitialized = true;
        }

        public ROS2Node()
        {
            CheckROSVersionSourced();
            CheckROSRMWSourced();
            CreateNode();
#if UNITY_EDITOR
            EditorApplication.playModeStateChanged += this.EditorPlayStateChanged;
            EditorApplication.quitting += this.OnApplicationQuit;
#endif
        }

        private static void ThrowIfUninitialized(string callContext)
        {
            if (!isInitialized)
                throw new InvalidOperationException("Context is not initialized, can't " + callContext);
        }

        /// <summary>
        /// Check if ROS2 node is properly initialized and no shutdown was called yet
        /// </summary>
        /// <returns>The state of ROS2 node. Should be checked before attempting to create or use pubs/subs</returns>
        public bool Ok()
        {
            if (!isInitialized)
                return false;
            return Ros2cs.Ok();
        }

        /// <summary>
        /// Primary loop that performs a single executor spin for ROS2, getting messages into subscribers
        /// </summary>
        public void SpinOnce()
        {
            if (!isInitialized)
                return;

            Ros2cs.SpinOnce(node, timeout);
        }

        /// <summary>
        /// Create a publisher with QoS suitable for sensor data
        /// </summary>
        /// <returns>The publisher</returns>
        /// <param name="topicName">topic that will be used for publishing</param>
        public Publisher<T> CreateSensorPublisher<T>(string topicName) where T : MessageWithHeader, IMessageInternals, new()
        {
            QualityOfServiceProfile sensorProfile = new QualityOfServiceProfile(QosProfiles.SENSOR_DATA);
            return CreatePublisher<T>(topicName, sensorProfile);
        }

        /// <summary>
        /// Create a publisher with indicated QoS.
        /// </summary>
        /// <returns>The publisher</returns>
        /// <param name="topicName">topic that will be used for publishing</param>
        /// <param name="qos">QoS for publishing. If no QoS is selected, it will default to reliable, keep 10 last</param>
        public Publisher<T> CreatePublisher<T>(string topicName, QualityOfServiceProfile qos = null) where T : Message, IMessageInternals, new()
        {
            ThrowIfUninitialized("create publisher");
            return node.CreatePublisher<T>(topicName, qos);
        }

        /// <summary>
        /// Create a subscription
        /// </summary>
        /// <returns>The subscription</returns>
        /// <param name="topicName">topic to subscribe to</param>
        /// <param name="qos">QoS for subscription. If no QoS is selected, it will default to reliable, keep 10 last</param>
        public Subscription<T> CreateSubscription<T>(string topicName, Action<T> callback,
            QualityOfServiceProfile qos = null) where T : Message, IMessageInternals, new()
        {
            if (qos == null)
                qos = new QualityOfServiceProfile(QosProfiles.DEFAULT);
            ThrowIfUninitialized("create subscription");
            return node.CreateSubscription<T>(topicName, callback, qos);
        }

        public void DestroyNode()
        {
            if (isInitialized)
            {
                isInitialized = false;
                node.Dispose();
                Ros2cs.Shutdown();
            }
        }

        void OnApplicationQuit()
        {
            DestroyNode();
        }

        ~ROS2Node()
        {
            DestroyNode();
        }

#if UNITY_EDITOR
        void EditorPlayStateChanged(PlayModeStateChange change)
        {
            if (change == PlayModeStateChange.ExitingPlayMode)
            {
                DestroyNode();
            }
        }
#endif
    }
}