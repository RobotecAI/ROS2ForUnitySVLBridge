
using System;
using System.Collections.Generic;
using System.Linq;
using Simulator.Bridge.Data;
using Unity.Mathematics;
using UnityEngine;

namespace Simulator.Bridge
{
    static class Ros2NativeConversions
    {
        public static builtin_interfaces.msg.Time ConvertTime(double unixEpochSeconds)
        {
            long nanosec = (long)(unixEpochSeconds * 1e9);

            return new builtin_interfaces.msg.Time()
            {
                Sec = (int)(nanosec / 1000000000),
                Nanosec = (uint)(nanosec % 1000000000),
            };
        }

        public static double Convert(builtin_interfaces.msg.Time time)
        {
            return (double)time.Sec + (double)time.Nanosec * 1e-9;
        }

        static geometry_msgs.msg.Quaternion Convert(UnityEngine.Quaternion q)
        {
            return new geometry_msgs.msg.Quaternion() { X = q.x, Y = q.y, Z = q.z, W = q.w };
        }

        static geometry_msgs.msg.Vector3 Convert(UnityEngine.Vector3 v)
        {
            return new geometry_msgs.msg.Vector3() { X = v.x, Y = v.y, Z = v.z };
        }

        public static VehicleControlData ConvertTo(lgsvl_msgs.msg.VehicleControlData data)
        {
            float Deg2Rad = UnityEngine.Mathf.Deg2Rad;
            float MaxSteeringAngle = 39.4f * Deg2Rad;
            float wheelAngle = 0f;

            if (data.Target_wheel_angle > MaxSteeringAngle)
            {
                wheelAngle = MaxSteeringAngle;
            }
            else if (data.Target_wheel_angle < -MaxSteeringAngle)
            {
                wheelAngle = -MaxSteeringAngle;
            }

            // ratio between -MaxSteeringAngle and MaxSteeringAngle
            var k = (float)(wheelAngle + MaxSteeringAngle) / (MaxSteeringAngle*2);

            // target_wheel_angular_rate, target_gear are not supported on simulator side.

            return new VehicleControlData()
            {
                TimeStampSec = Convert(data.Header.Stamp),
                Acceleration = data.Acceleration_pct,
                Braking = data.Braking_pct,
                SteerAngle = UnityEngine.Mathf.Lerp(-1f, 1f, k),

            };
        }

        public static VehicleStateData ConvertTo(lgsvl_msgs.msg.VehicleStateData data)
        {
            return new VehicleStateData()
            {
                Time = Convert(data.Header.Stamp),
                Blinker = (byte) data.Blinker_state,
                HeadLight = (byte) data.Headlight_state,
                Wiper = (byte) data.Wiper_state,
                Gear = (byte) data.Current_gear,
                Mode = (byte) data.Vehicle_mode,
                HandBrake = data.Hand_brake_active,
                Horn = data.Horn_active,
                Autonomous = data.Autonomous_mode_active,
            };
        }

        public static sensor_msgs.msg.CompressedImage ConvertFrom(ImageData data)
        {
            var time = ConvertTime(data.Time);
            var msg = new sensor_msgs.msg.CompressedImage()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Format = "jpeg"
            };
            msg.Data = new byte[data.Length]; //TODO (piotr.jarszek) initialize only when data len changes
            System.Buffer.BlockCopy(data.Bytes, 0, msg.Data, 0, data.Length);
            return msg;
        }

        public static rosgraph_msgs.msg.Clock ConvertFrom(ClockData data)
        {
            var time = ConvertTime(data.Clock);
            var msg = new rosgraph_msgs.msg.Clock();

            msg.Clock_.Sec = (int)time.Sec;
            msg.Clock_.Nanosec = time.Nanosec;

            return msg;
        }

        public static nav_msgs.msg.Odometry ConvertFrom(GpsOdometryData data)
        {
            var time = ConvertTime(data.Time);
            var orientation = Convert(data.Orientation);
            var msg = new nav_msgs.msg.Odometry() {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Child_frame_id = data.ChildFrame,
                Pose = new geometry_msgs.msg.PoseWithCovariance()
                {
                    Pose = new geometry_msgs.msg.Pose()
                    {
                        Position = new geometry_msgs.msg.Point()
                        {
                            X = data.Easting + (data.IgnoreMapOrigin ? -500000 : 0),
                            Y = data.Northing,
                            Z = data.Altitude,
                        },
                        Orientation = orientation
                    }
                },
                Twist = new geometry_msgs.msg.TwistWithCovariance()
                {
                    Twist = new geometry_msgs.msg.Twist()
                    {
                        Linear = new geometry_msgs.msg.Vector3()
                        {
                            X = data.ForwardSpeed,
                            Y = 0.0,
                            Z = 0.0,
                        },
                        Angular = new geometry_msgs.msg.Vector3()
                        {
                            X = 0.0,
                            Y = 0.0,
                            Z = - data.AngularVelocity.y,
                        }
                    },
                }
            };

            for (int i = 0; i < 36; i++)
            {
                if (i%7 == 0)
                {
                    msg.Pose.Covariance[i] = (0.0001);
                    msg.Twist.Covariance[i] = (0.0001);
                } else {
                    msg.Pose.Covariance[i] = (0);
                    msg.Twist.Covariance[i] = (0);
                }
            }

            return msg;
        }

        public static lgsvl_msgs.msg.CanBusData ConvertFrom(CanBusData data)
        {
            var time = ConvertTime(data.Time);
            var orientation = Convert(data.Orientation);
            var velocities = Convert(data.Velocity);
            return new lgsvl_msgs.msg.CanBusData()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Fog_lights_active = data.FogLights,
                Left_turn_signal_active = data.LeftTurnSignal,
                Right_turn_signal_active = data.RightTurnSignal,
                Wipers_active = data.Wipers,
                Selected_gear = (sbyte)data.Gear,
                Gps_latitude = data.Latitude,
                Gps_longitude = data.Longitude,
                Gps_altitude = data.Altitude,
                Engine_rpm = data.EngineRPM,
                Hazard_lights_active = data.HazardLights,
                Orientation = orientation,
                Linear_velocities = velocities,
                Engine_active = data.EngineOn,
                Low_beams_active = data.LowBeamSignal,
                Steer_pct = data.Steering,
                Parking_brake_active = data.ParkingBrake,
                Throttle_pct = data.Throttle,
                Speed_mps = data.Speed,
                High_beams_active = data.HighBeamSignal
            };
        }

        public static sensor_msgs.msg.Imu ConvertFrom(ImuData data)
        {
            var time = ConvertTime(data.Time);
            var orientation = Convert(data.Orientation);
            var imu = new sensor_msgs.msg.Imu()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },

                Orientation = orientation,
                Angular_velocity = new geometry_msgs.msg.Vector3() { X = data.AngularVelocity.z, Y = -data.AngularVelocity.x, Z = data.AngularVelocity.y },
                Linear_acceleration = new geometry_msgs.msg.Vector3() { X = data.Acceleration.z, Y = -data.Acceleration.x, Z = data.Acceleration.y }
            };

            for (int i = 0; i < 9; i++)
            {
                if(i%4 == 0) {
                    imu.Orientation_covariance[i] = (0.0001);
                    imu.Angular_velocity_covariance[i] = (0.0001);
                    imu.Linear_acceleration_covariance[i] = (0.0001);
                } else {
                    imu.Orientation_covariance[i] = (0);
                    imu.Angular_velocity_covariance[i] = (0);
                    imu.Linear_acceleration_covariance[i] = (0);
                }
            }

            return imu;
        }

       public static sensor_msgs.msg.NavSatFix ConvertFrom(GpsData data)
        {
            var time = ConvertTime(data.Time);
            var msg = new sensor_msgs.msg.NavSatFix()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Status = new sensor_msgs.msg.NavSatStatus()
                {
                    Status = sensor_msgs.msg.NavSatStatus.STATUS_FIX,
                    Service = sensor_msgs.msg.NavSatStatus.SERVICE_GPS,
                },
                Latitude = data.Latitude,
                Longitude = data.Longitude,
                Altitude = data.Altitude,

                Position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            };
            for (int i = 0; i < 9; i++)
            {
                if (i%4 == 0)
                {
                    msg.Position_covariance[i] = (0.0001);
                } else {
                    msg.Position_covariance[i] = (0);
                }
            }
            return msg;
        } 
    }
}