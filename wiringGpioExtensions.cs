

using System;
using System.Runtime.InteropServices;
using static wiringGpioExtensions.Library;


namespace wiringGpioExtensions
{
    public static class Library
    {
        public const string LibName = "wiringGpioExtensions";
    }
    

    public static class Constants
    {
        public static double RPiPwmClockSpeed = 19.2e6;
        public static int RPiPwmFrequency = 50;
    }

    public enum PinMode
    {
        Input = 0,
        Output = 1,
        PWMOutput = 2,
        GPIOClock = 3,
        SoftPwmOutput,
        SoftToneOutput,
        PwmToneOutput,
    }

    public enum PinValue
    {
        Low = 0,
        High = 1
    }

    public enum PullUpDownValue
    {
        Off = 0,
        Down = 1,
        Up = 2
    }

#if JETSON
    //  Interrupt level defines are swaped on Jetson
    public enum InterruptLevels
    {
        INT_EDGE_SETUP = 0,
        INT_EDGE_FALLING = 2,
        INT_EDGE_RISING = 1,
        INT_EDGE_BOTH = 3
    }
#else
 public enum InterruptLevels
    {
        INT_EDGE_SETUP = 0,
        INT_EDGE_FALLING = 1,
        INT_EDGE_RISING = 2,
        INT_EDGE_BOTH = 3
    }
#endif

    public enum Mcp23Type
    {
        Mcp23017 = 0,
        Mcp23008 = 1,
    }


    public struct WiringGpioLogEvent
    {
        public long LogUnixTimeMilliseconds;
        public Logging.LogLevel Level;
        public int Thread;
        public IntPtr SenderNamePtr;
        public IntPtr FunctionNamePtr;
        public IntPtr DataPtr;
        
    }



   

    /// <summary>
    /// Initialization
    /// Raw functions can be used to initialise Gordon's library if you only want to use raw functions from wiringPi
    /// If you are using the extension, you should call SetupWiringPiExtension
    /// </summary>
    public class Setup
    {
        [DllImport(LibName, EntryPoint = "WiringGpioSetup")]
        public static extern int WiringGpioSetup();

        [DllImport(LibName, EntryPoint = "WiringGpioSetupGpio")]
        public static extern int WiringGpioSetupGpio();

        [DllImport(LibName, EntryPoint = "WiringGpioSetupSys")]
        public static extern int WiringGpioSetupSys();

        [DllImport(LibName, EntryPoint = "WiringGpioSetupPhys")]
        public static extern int WiringGpioSetupPhys();

        [DllImport(LibName, EntryPoint = "WiringGpioTerminate")]
        public static extern void WiringGpioTerminate();
    }


    /// <summary>
    /// GPIO pin functions
    /// </summary>
    public class GPIO
    {
        //  Basic GPIO
        [DllImport(LibName, EntryPoint = "PinModeAlt")]
        public static extern void PinModeAlt(int pin, PinMode mode);

        [DllImport(LibName, EntryPoint = "PinMode")]           
        public static extern void PinMode(int pin, PinMode mode);

        [DllImport(LibName, EntryPoint = "PullUpDnControl")]
        public static extern void PullUpDnControl(int pin, PullUpDownValue pud);

        [DllImport(LibName, EntryPoint = "DigitalRead")]
        public static extern int DigitalRead(int pin);

        [DllImport(LibName, EntryPoint = "DigitalWrite")]      
        public static extern void DigitalWrite(int pin, PinValue value);

        [DllImport(LibName, EntryPoint = "AnalogRead")]
        public static extern int AnalogRead(int pin);

        [DllImport(LibName, EntryPoint = "AnalogWrite")]
        public static extern void AnalogWrite(int pin, int value);

        //  PWM
        [DllImport(LibName, EntryPoint = "PwmWrite")]
        public static extern void PwmWrite(int pin, int value);

        [DllImport(LibName, EntryPoint = "PwmWriteUnit")]
        public static extern void PwmWriteUnit(int pin, float value);

        //  Hardware PWM
        [DllImport(LibName, EntryPoint = "GpioClockSet")]
        public static extern void GpioClockSet(int pin, int freq);

        [DllImport(LibName, EntryPoint = "PwmSetMode")]             
        public static extern void PwmSetMode(PinMode mode);
        
        [DllImport(LibName, EntryPoint = "PwmSetClock")]             
        public static extern void PwmSetClock(int divisor);

        [DllImport(LibName, EntryPoint = "PwmSetFrequency")]
        public static extern void PwmSetFrequency(int pin, float frequency);

        [DllImport(LibName, EntryPoint = "PwmGetFrequency")]
        public static extern float PwmGetFrequency(int pin);

        [DllImport(LibName, EntryPoint = "PwmSetRange")]
        public static extern void PwmSetRange(int range);

        [DllImport(LibName, EntryPoint = "PwmGetRange")]
        public static extern int PwmGetRange(int pin);

        [DllImport(LibName, EntryPoint = "PwmIsHardwarePwmPin")]
        public static extern int PwmIsHardwarePwmPin(int pin);
    }


    /// <summary>
    /// Software PWM
    /// </summary>
    public class SoftwarePwm
    {
        //  Software PWM
        [DllImport(LibName, EntryPoint = "SoftPwmCreate")]
        public static extern int SoftPwmCreate(int pin, int value, int range);

        [DllImport(LibName, EntryPoint = "SoftPwmWrite")]
        public static extern void SoftPwmWrite(int pin, int value);

        [DllImport(LibName, EntryPoint = "SoftPwmStop")]
        public static extern void SoftPwmStop(int pin);
    }
    
    

   


    /// <summary>
    /// Provides access to the Thread priority and interrupts for IO
    /// </summary>
    public delegate void ISRCallback();
    //
    public class Interrupts
    {
        [DllImport(LibName, EntryPoint = "WaitForInterrupt")]
        public static extern int WaitForInterrupt(int pin, int timeout);
        
        [DllImport(LibName, EntryPoint = "WiringGpioISR")]
        public static extern int WiringGpioISR(int pin, int mode, ISRCallback method);

    }


    /// <summary>
    /// Provides SPI port functionality
    /// </summary>
    public class SPI
    {
        /// <summary>
        /// Configures the SPI channel 
        /// </summary>
        /// <param name="channel">Selects either Channel 0 or 1 for use</param>
        /// <param name="speed">Selects speed, 500,000 to 32,000,000</param>
        /// <returns>-1 for an error, or the linux file descriptor the channel uses</returns>
        [DllImport(LibName, EntryPoint = "WiringGpioSPISetup")]
        public static extern int WiringGpioSPISetup(int channel, int speed);

        /// <summary>
        /// Read and Write data over the SPI bus, don't forget to configure it first
        /// </summary>
        /// <param name="channel">Selects Channel 0 or Channel 1 for this operation</param>
        /// <param name="data">signed byte array pointer which holds the data to send and will then hold the received data</param>
        /// <param name="len">How many bytes to write and read</param>
        /// <returns>-1 for an error, or the linux file descriptor the channel uses</returns>
        [DllImport(LibName, EntryPoint = "WiringGpioSPIDataRW")]
        public static unsafe extern int WiringGpioSPIDataRW(int channel, byte* data, int len);  //char is a signed byte
    }


    /// <summary>
    /// Provides access to the I2C port
    /// </summary>
    public class I2C
    {
        [DllImport(LibName, EntryPoint = "WiringGpioI2CSetup")]
        public static extern int WiringGpioI2CSetup(int bus, int devId);

        [DllImport(LibName, EntryPoint = "WiringGpioI2CWrite")]
        public static extern int WiringGpioI2CWrite(int fd, int data);

        [DllImport(LibName, EntryPoint = "WiringGpioI2CRead")]
        public static extern int WiringGpioI2CRead(int fd);

        [DllImport(LibName, EntryPoint = "WiringGpioI2CWriteReg8")]
        public static extern int WiringGpioI2CWriteReg8(int fd, int reg, int data);

        [DllImport(LibName, EntryPoint = "WiringGpioI2CWriteReg16")]
        public static extern int WiringGpioI2CWriteReg16(int fd, int reg, int data);

        [DllImport(LibName, EntryPoint = "WiringGpioI2CReadReg8")]
        public static extern int WiringGpioI2CReadReg8(int fd, int reg);

        [DllImport(LibName, EntryPoint = "WiringGpioI2CReadReg16")]
        public static extern int WiringGpioI2CReadReg16(int fd, int reg);
    }



    /// <summary>
    /// MCP 23008 and 23017 Pin Expander Chips
    /// </summary>
    public class DeviceI2CExtensions
    {
        //  MCP23008
        [DllImport(LibName, EntryPoint = "Mcp23008Setup")]
        public static extern int Mcp23008Setup(int bus, int pinBase, int address);

        //  MCP23017
        [DllImport(LibName, EntryPoint = "Mcp23017Setup")]
        public static extern int Mcp23017Setup(int bus, int pinBase, int address);

        //  PCA9685
        [DllImport(LibName, EntryPoint = "Pca9685Setup")]
        public static extern int Pca9685Setup(int bus, int pinBase, int i2cAddress, float freq);
        //
        [DllImport(LibName, EntryPoint = "Pca9685PWMReset")]
        public static extern void Pca9685PWMReset(int fd);
        //
        [DllImport(LibName, EntryPoint = "Pca9685FullOn")]
        public static extern void Pca9685FullOn(int fd, int pin, int tf);
        //
        [DllImport(LibName, EntryPoint = "Pca9685FullOff")]
        public static extern void Pca9685FullOff(int fd, int pin, int tf);
    }


    public class DeviceSpiExtensions
    {
        [DllImport(LibName, EntryPoint = "Mcp3004Setup")]
        public static extern int Mcp3004Setup(int pinBase, int spiChannel);

        [DllImport(LibName, EntryPoint = "Mcp3008Setup")]
        public static extern int Mcp3008Setup(int pinBase, int spiChannel);
    }


    /// <summary>
    /// Software stepper motor
    /// </summary>
    public class StepperMotor
    {
        [DllImport(LibName, EntryPoint = "StepperCreateFromXml")]
        public static extern int CreateFromXml(string sequenceElement, string pinsElement);

        [DllImport(LibName, EntryPoint = "StepperRemove")]
        public static extern void Remove(int index);

        [DllImport(LibName, EntryPoint = "StepperSetDelay")]
        public static extern void SetDelay(int index, float delay);

        [DllImport(LibName, EntryPoint = "StepperStep")]
        public static extern void Step(int index, int numberSteps);

        [DllImport(LibName, EntryPoint = "StepperSpin")]
        public static extern void Spin(int index, int direction);

        [DllImport(LibName, EntryPoint = "StepperSetSpeed")]
        public static extern void SetSpeed(int index, float percent);

        [DllImport(LibName, EntryPoint = "StepperSetSequenceInterval")]
        public static extern void SetSequenceInterval(int index, int interval);

        [DllImport(LibName, EntryPoint = "StepperStop")]
        public static extern void Stop(int index);

        [DllImport(LibName, EntryPoint = "StepperGetTachoCount")]
        public static extern int GetTachoCount(int index);

        [DllImport(LibName, EntryPoint = "StepperResetTachoCount")]
        public static extern void ResetTachoCount(int index);

        [DllImport(LibName, EntryPoint = "StepperShutDown")]
        public static extern void ShutDown();
    }


    /// <summary>
    /// Seven Segment Display
    /// </summary>
    public class SevenSegDisplay
    {
        [DllImport(LibName, EntryPoint = "SevenSegDisplayCreateFromXml")]
        public static extern int CreateFromXml(string xml);

        [DllImport(LibName, EntryPoint = "SevenSegDisplayCreate")]
        public static extern int Create(int[] segPins, int numDigits, int[] digitPins);

        [DllImport(LibName, EntryPoint = "SevenSegDisplayRemove")]
        public static extern void Remove(int index);

        [DllImport(LibName, EntryPoint = "SevenSegDisplayOff")]
        public static extern void Off(int index);

        [DllImport(LibName, EntryPoint = "SevenSegDisplaySetDelay")]
        public static extern void SetDelay(int index, int delay);

        [DllImport(LibName, EntryPoint = "SevenSegDisplaySet")]
        public static extern void Set(int index, string display);

        [DllImport(LibName, EntryPoint = "SevenSegDisplayShutDown")]
        public static extern void ShutDown();
    }


    /// <summary>
    /// Rotary Encoder
    /// </summary>
    //
    public delegate void EncoderUpdatedCallback(int count);
    //
    public class RotaryEncoder
    {
        [DllImport(LibName, EntryPoint = "RotaryEncoderCreate")]
        public static extern int Create(int pinA, int pinB, int pinIndex, int countsPerRevolution, EncoderUpdatedCallback callback);

        [DllImport(LibName, EntryPoint = "RotaryEncoderRemove")]
        public static extern void Remove(int index);

        [DllImport(LibName, EntryPoint = "RotaryEncoderGetCount")]
        public static extern int GetCount(int index);

        [DllImport(LibName, EntryPoint = "RotaryEncoderGetRpm")]
        public static extern double GetRpm(int index);

        [DllImport(LibName, EntryPoint = "RotaryEncoderGetFrequency")]
        public static extern double GetFrequency(int index);

        [DllImport(LibName, EntryPoint = "RotaryEncoderResetCount")]
        public static extern void ResetCount(int index, int setCount);

        [DllImport(LibName, EntryPoint = "RotaryEncoderShutDown")]
        public static extern void ShutDown();
    }


    /// <summary>
    /// Motor with Rotary Encoder
    /// </summary>
    public class MotorWithRotaryEncoder
    {
        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderCreate")]
        public static extern int Create(int bridgeIn1, int bridgeIn2, int bridgePwm, int encoderA, int encoderB, int encoderIndex, int countsPerRevolution, EncoderUpdatedCallback callback);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderRemove")]
        public static extern void Remove(int index);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderSetUsefulPowerRange")]
        public static extern void SetUsefulPowerRange(int index, double minPower, double maxPower);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderResetCount")]
        public static extern void ResetCount(int index, int setCount);
        
        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderGetCount")]
        public static extern int GetCount(int index);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderGetTick")]
        public static extern int GetTick(int index);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderGetCircle")]
        public static extern double GetCircle(int index);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderGetRpm")]
        public static extern double GetRpm(int index);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderGetTickFrequency")]
        public static extern double GetTickFrequency(int index);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderGetFrequency")]
        public static extern double GetFrequency(int index);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderRun")]
        public static extern void Run(int index, double power);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderStop")]
        public static extern void Stop(int index);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderBrake")]
        public static extern void Brake(int index, double power);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderTurnBy")]
        public static extern void TurnBy(int index, double rotations, double power);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderHoldAt")]
        public static extern void HoldAt(int index, double circle, double power);

        [DllImport(LibName, EntryPoint = "MotorWithRotaryEncoderShutDown")]
        public static extern void ShutDown();
    }


    /// <summary>
    /// Extension node management
    /// Convenience functions to get pin base or fd of a node containing a given pin
    /// </summary>
    public class ExtensionNodeManagement
    {
        [DllImport(LibName, EntryPoint = "WiringGpioGetPinBaseForNode")]
        public static extern int WiringGpioGetPinBaseForNode(int pin);

        [DllImport(LibName, EntryPoint = "WiringGpioGetFileDescriptorForNode")]
        public static extern int WiringGpioGetFileDescriptorForNode(int pin);
    }


    /// <summary>
    /// Logging
    /// </summary>
    /// 
    public delegate void LoggingCallback(WiringGpioLogEvent e);
    //
    public class Logging
    {
        public enum LogLevel
        {
            All = 0,
            Trace,
            Debug,
            Info,
            Warn,
            Error,
            Fatal,
            Off,
        }

        [DllImport(LibName, EntryPoint = "WiringGpioSetLoggingCallback")]
        public static extern void WiringGpioSetLoggingCallback(LoggingCallback fn);

        [DllImport(LibName, EntryPoint = "WiringGpioSetLoggingLevel")]
        public static extern void WiringGpioSetLoggingLevel(LogLevel level);

    }
}