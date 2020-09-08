
using System;
using System.Collections.Generic;
using System.Linq;
using Simulator.Bridge.Data;

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