using System;
using static System.Runtime.InteropServices.Marshal;

namespace wiringGpioExtensions
{

    public class LogEvent
    {
        public static DateTime UnixTimeStampToDateTime(long unixTimeStamp)
        {
            // Unix timestamp is seconds past epoch
            DateTime dateTime = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);
            dateTime = dateTime.AddMilliseconds(unixTimeStamp).ToLocalTime();
            return dateTime;
        }


        public LogEvent(WiringGpioLogEvent e)
        {
            LogTime = UnixTimeStampToDateTime(e.LogUnixTimeMilliseconds);
            ObjectName = PtrToStringAnsi(e.SenderNamePtr);
            FunctionName = PtrToStringAnsi(e.FunctionNamePtr);
            Log = PtrToStringAnsi(e.DataPtr);
            Level = e.Level;
        }

        public DateTimeOffset LogTime { get; protected set; }
        public string ObjectName { get; protected set; }
        public string FunctionName { get; protected set; }
        public string Log { get; protected set; }
        public Logging.LogLevel Level;


    }
}
