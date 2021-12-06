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
using System.Collections.Generic;
using System.Linq;
using Simulator.Bridge.Data;
using Unity.Mathematics;
using UnityEngine;

namespace Simulator.Bridge
{
    static class ROS2ForUnitySVLBridgeConversions
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

        static geometry_msgs.msg.Point ConvertToPoint(UnityEngine.Vector3 v)
        {
            return new geometry_msgs.msg.Point() { X = v.x, Y = v.y, Z = v.z };
        }

        static geometry_msgs.msg.Vector3 ConvertToVector(UnityEngine.Vector3 v)
        {
            return new geometry_msgs.msg.Vector3() { X = v.x, Y = v.y, Z = v.z };
        }

        static geometry_msgs.msg.Vector3 Convert(UnityEngine.Vector3 v)
        {
            return new geometry_msgs.msg.Vector3() { X = v.x, Y = v.y, Z = v.z };
        }

        static UnityEngine.Vector3 Convert(geometry_msgs.msg.Point p)
        {
            return new UnityEngine.Vector3((float)p.X, (float)p.Y, (float)p.Z);
        }

        static UnityEngine.Quaternion Convert(geometry_msgs.msg.Quaternion q)
        {
            return new UnityEngine.Quaternion((float)q.X, (float)q.Y, (float)q.Z, (float)q.W);
        }

        static UnityEngine.Vector3 Convert(geometry_msgs.msg.Vector3 v)
        {
            return new UnityEngine.Vector3((float)v.X, (float)v.Y, (float)v.Z);
        }


        public static VehicleControlData ConvertTo(lgsvl_msgs.msg.VehicleControlData data)
        {
            float Deg2Rad = UnityEngine.Mathf.Deg2Rad;
            float MaxSteeringAngle = 39.4f * Deg2Rad;
            float wheelAngle = data.Target_wheel_angle;

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
                TargetGear = (GearPosition)(int)data.Target_gear
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
                Gear = (byte)data.Current_gear,
                Mode = (byte) data.Vehicle_mode,
                HandBrake = data.Hand_brake_active,
                Horn = data.Horn_active,
                Autonomous = data.Autonomous_mode_active,
            };
        }

        public static Detected2DObjectArray ConvertTo(lgsvl_msgs.msg.Detection2DArray data)
        {
            return new Detected2DObjectArray()
            {
                Data = data.Detections.Select(obj =>
                    new Detected2DObject()
                    {
                        Id = obj.Id,
                        Label = obj.Label,
                        Score = obj.Score,
                        Position = new UnityEngine.Vector2(obj.Bbox.X, obj.Bbox.Y),
                        Scale = new UnityEngine.Vector2(obj.Bbox.Width, obj.Bbox.Height),
                        LinearVelocity = new UnityEngine.Vector3((float)obj.Velocity.Linear.X, 0, 0),
                        AngularVelocity = new UnityEngine.Vector3(0, 0, (float)obj.Velocity.Angular.Z),
                    }).ToArray(),
            };
        }

        public static Detected3DObjectArray ConvertTo(lgsvl_msgs.msg.Detection3DArray data)
        {
            return new Detected3DObjectArray()
            {
                Data = data.Detections.Select(obj =>
                    new Detected3DObject()
                    {
                        Id = obj.Id,
                        Label = obj.Label,
                        Score = obj.Score,
                        Position = Convert(obj.Bbox.Position.Position),
                        Rotation = Convert(obj.Bbox.Position.Orientation),
                        Scale = Convert(obj.Bbox.Size),
                        LinearVelocity = Convert(obj.Velocity.Linear),
                        AngularVelocity = Convert(obj.Velocity.Angular),
                    }).ToArray(),
            };
        }

        public static lgsvl_msgs.msg.DetectedRadarObjectArray ConvertFrom(DetectedRadarObjectData data)
        {
            var time = ROS2ForUnitySVLBridgeConversions.ConvertTime(data.Time);
            var msg = new lgsvl_msgs.msg.DetectedRadarObjectArray()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Objects = new lgsvl_msgs.msg.DetectedRadarObject[data.Data.Length]
            };

            int index = 0;
            foreach (var obj in data.Data)
            {
                msg.Objects[index] = new lgsvl_msgs.msg.DetectedRadarObject
                {
                    Sensor_aim = ConvertToVector(obj.SensorAim),
                    Sensor_right = ConvertToVector(obj.SensorRight),
                    Sensor_position = ConvertToPoint(obj.SensorPosition),
                    Sensor_velocity = ConvertToVector(obj.SensorVelocity),
                    Sensor_angle = obj.SensorAngle,
                    Object_position = ConvertToPoint(obj.Position),
                    Object_velocity = ConvertToVector(obj.Velocity),
                    Object_relative_position = ConvertToPoint(obj.RelativePosition),
                    Object_relative_velocity = ConvertToVector(obj.RelativeVelocity),
                    Object_collider_size = ConvertToVector(obj.ColliderSize),
                    Object_state = (byte)obj.State,
                    New_detection = obj.NewDetection,
                };
                index++;
            }

            return msg;
        }

        public static sensor_msgs.msg.LaserScan ConvertFrom(LaserScanData data)
        {
            var count = data.Points.Length;

            if (data.RangesCache == null || data.RangesCache.Length != count)
                data.RangesCache = new float[count];

            if (data.IntensitiesCache == null || data.IntensitiesCache.Length != count)
                data.IntensitiesCache = new float[count];

            for (var i = 0; i < count; ++i)
            {
                var point = data.Points[i];

                var pos = new UnityEngine.Vector3(point.x, point.y, point.z);
                var intensity = point.w * 255f;

                pos = data.Transform.MultiplyPoint3x4(pos);
                var distance = pos.magnitude;
                if (distance < data.RangeMin || distance > data.RangeMax)
                {
                    distance = float.PositiveInfinity;
                    intensity = 0;
                }

                var iOut = count - 1 - i;
                data.RangesCache[iOut] = distance;
                data.IntensitiesCache[iOut] = intensity;
            }

            var time = ROS2ForUnitySVLBridgeConversions.ConvertTime(data.Time);
            var msg = new sensor_msgs.msg.LaserScan()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Angle_min = data.MinAngle,
                Angle_max = data.MaxAngle,
                Angle_increment = data.AngleStep,
                Time_increment = data.TimeIncrement,
                Scan_time = data.ScanTime,
                Range_min = data.RangeMin,
                Range_max = data.RangeMax,
                Ranges = data.RangesCache,
                Intensities = data.IntensitiesCache
            };

            return msg;
        }

        public static sensor_msgs.msg.CameraInfo ConvertFrom(CameraInfoData data)
        {
            var time = ConvertTime(data.Time);
            var msg = new sensor_msgs.msg.CameraInfo() 
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Height = (uint)data.Height,
                Width = (uint)data.Width,
                Distortion_model = "plumb_bob",
                D = new double[5]
                {
                    (double)data.DistortionParameters[0],
                    (double)data.DistortionParameters[1],
                    0.0,
                    0.0,
                    (double)data.DistortionParameters[2],
                },
                Binning_x = 0,
                Binning_y = 0,
                Roi = new sensor_msgs.msg.RegionOfInterest()
                {
                    X_offset = 0,
                    Y_offset = 0,
                    Width = 0,
                    Height = 0,
                    Do_rectify = false,
                },
            };
            
            msg.K[0] = data.FocalLengthX;
            msg.K[2] = data.PrincipalPointX;
            msg.K[4] = data.FocalLengthY;
            msg.K[5] = data.PrincipalPointY;
            msg.K[8] = 1.0;

            msg.R[0] = 1.0;
            msg.R[4] = 1.0;
            msg.R[8] = 1.0;

            msg.P[0] = data.FocalLengthX;
            msg.P[2] = data.PrincipalPointX;
            msg.P[5] = data.FocalLengthY;
            msg.P[6] = data.PrincipalPointY;
            msg.P[10] = 1.0;
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
            
            sbyte gear;
            if (data.InReverse) {
                gear = (sbyte)GearPosition.Reverse;
            } else {
                gear = (sbyte)GearPosition.Drive;
            }

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
                Selected_gear = gear,
                Reverse_gear_active = data.InReverse,
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
                Angular_velocity = new geometry_msgs.msg.Vector3() { X = data.AngularVelocity.x, Y = data.AngularVelocity.y, Z = data.AngularVelocity.z },
                Linear_acceleration = new geometry_msgs.msg.Vector3() { X = data.Acceleration.x, Y = data.Acceleration.y, Z = data.Acceleration.z }
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

        public static lgsvl_msgs.msg.VehicleOdometry ConvertFrom(VehicleOdometryData data)
        {
            float Deg2Rad = UnityEngine.Mathf.Deg2Rad;
            var time = ConvertTime(data.Time);
            var odom = new lgsvl_msgs.msg.VehicleOdometry()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time
                },

                Velocity = data.Speed,
                Front_wheel_angle = data.SteeringAngleFront * Deg2Rad,
                Rear_wheel_angle = data.SteeringAngleBack * Deg2Rad
            };

            return odom;
        }

        public static lgsvl_msgs.msg.Detection2DArray ConvertFrom(Detected2DObjectData data)
        {
            var time = ConvertTime(data.Time);
            var arr = new lgsvl_msgs.msg.Detection2DArray()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Detections = new lgsvl_msgs.msg.Detection2D[data.Data.Length],
            };
            
            int index = 0;
            foreach (var d in data.Data) {
                var detection = new lgsvl_msgs.msg.Detection2D()
                {
                    Id = d.Id,
                    Label = d.Label,
                    Score = (float)d.Score,
                    Bbox = new lgsvl_msgs.msg.BoundingBox2D()
                    {
                        X = d.Position.x,
                        Y = d.Position.y,
                        Width = d.Scale.x,
                        Height = d.Scale.y
                    },
                    Velocity = new geometry_msgs.msg.Twist()
                    {
                        Linear = ConvertToVector(d.LinearVelocity),
                        Angular = ConvertToVector(d.AngularVelocity),
                    }
                };
                arr.Detections[index] = detection;
                index += 1;
            }

            return arr;
        }

        public static lgsvl_msgs.msg.Detection3DArray ConvertFrom(Detected3DObjectData data)
        {
            var time = ConvertTime(data.Time);
            var arr = new lgsvl_msgs.msg.Detection3DArray()
            {
                Header = new std_msgs.msg.Header()
                {
                    Stamp = time,
                    Frame_id = data.Frame,
                },
                Detections = new lgsvl_msgs.msg.Detection3D[data.Data.Length],
            };
            
            int index = 0;
            foreach (var d in data.Data)
            {
                // Transform from (Right/Up/Forward) to (Forward/Left/Up)
                var position = d.Position;
                position.Set(position.z, -position.x, position.y);

                var orientation = d.Rotation;
                orientation.Set(-orientation.z, orientation.x, -orientation.y, orientation.w);

                var size = d.Scale;
                size.Set(size.z, size.x, size.y);

                d.AngularVelocity.z = -d.AngularVelocity.z;

                var det = new lgsvl_msgs.msg.Detection3D()
                {
                    Id = d.Id,
                    Label = d.Label,
                    Score = (float)d.Score,
                    Bbox = new lgsvl_msgs.msg.BoundingBox3D()
                    {
                        Position = new geometry_msgs.msg.Pose()
                        {
                            Position = ConvertToPoint(position),
                            Orientation = Convert(orientation),
                        },
                        Size = ConvertToVector(size),
                    },
                    Velocity = new geometry_msgs.msg.Twist()
                    {
                        Linear = ConvertToVector(d.LinearVelocity),
                        Angular = ConvertToVector(d.AngularVelocity),
                    },
                };

                arr.Detections[index] = det;
                index += 1;
            }

            return arr;
        }
    }
}
