
using System;
using System.Collections.Generic;
using System.Linq;
using Simulator.Bridge.Data;
using Unity.Mathematics;

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

        static geometry_msgs.msg.Quaternion Convert(UnityEngine.Quaternion q)
        {
            return new geometry_msgs.msg.Quaternion() { X = q.x, Y = q.y, Z = q.z, W = q.w };
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

        public static autoware_auto_msgs.msg.VehicleStateReport ConvertFrom(CanBusData data)
        {

            // No fuel supported in Simulator side
            byte fuel = 0;

            // Blinker
            // BLINKER_OFF = 0, BLINKER_LEFT = 1, BLINKER_RIGHT = 2, BLINKER_HAZARD = 3
            // No Hazard Light in Simulator side
            byte blinker = 0;
            if (data.HazardLights)
                blinker = 3;
            else if (data.LeftTurnSignal)
                blinker = 1;
            else if (data.RightTurnSignal)
                blinker = 2;

            // Headlight
            // HEADLIGHT_OFF = 0, HEADLIGHT_ON = 1, HEADLIGHT_HIGH = 2
            byte headlight = 0;
            if (data.LowBeamSignal)
                headlight = 1;
            else if (data.HighBeamSignal)
                headlight = 2;
            else
                headlight = 0;

            // Wiper
            // WIPER_OFF = 0, WIPER_LOW = 1, WIPER_HIGH = 2, WIPER_CLEAN = 3
            // No WIPER_HIGH and WIPER_CLEAN in Simulator side
            byte wiper = 0;
            if (data.Wipers)
                wiper = 1;

            // Gear
            // GEAR_DRIVE = 0, GEAR_REVERSE = 1, GEAR_PARK = 2, GEAR_LOW = 3, GEAR_NEUTRAL = 4
            // No GEAR_PARK, GEAR_LOW, GEAR_NEUTRAL in Simulator side
            byte gear = 0;
            if (data.InReverse)
                gear = 1;
            else
                gear = 0;

            // Mode
            // No information about mode in Simulator side.
            byte mode = 0;

            // Hand Brake
            bool handBrake = false;

            // Horn
            bool horn = false;
            var time = ConvertTime(data.Time);
            return new autoware_auto_msgs.msg.VehicleStateReport()
            {
                Stamp = time,
                Fuel = fuel,
                Blinker = blinker,
                Headlight = headlight,
                Wiper = wiper,
                Gear = gear,
                Mode = mode,
                Hand_brake = handBrake,
                Horn = horn,
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