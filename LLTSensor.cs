using MEScanControl;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace LLT
{
    public class ScanPoint
    {
        public double X { get; set; }
        public double Z { get; set; }
    }

    public class Profile
    {
        private List<ScanPoint> Points { get; set; }

        public Profile(List<ScanPoint> points)
        {
            Points = points;
        }

        public int Count
        {
            get { return Points.Count; }
        }

        public ScanPoint this[int index]
        {
            get { return Points[index]; }
        }

        public ScanPoint[] ToArray()
        {
            return Points.ToArray();
        }
        public List<ScanPoint> ToList()
        {
            return Points.ToList();
        }

        public void Clear()
        {
            Points.Clear();
        }

        public void Add(ScanPoint point)
        {
            Points.Add(point);
        }
    }
    public class LLTSensor : CLLTI
    {
        #region Parameters
        private const int MAX_RESOLUTIONS = 6;

        // Handle returned by CreateLLTDevice
        public uint device { get; private set; }

        // Most recent human-readable error message
        public string ErrorMessage { get; private set; }

        public enum LLT30x430Resolutions
        {
            R256 = 256,
            R512 = 512,
            R1024 = 1024,
            R2048 = 2048
        }


        private LLT30x430Resolutions _resolution = LLT30x430Resolutions.R512;
        private const LLT30x430Resolutions DEFAULT_RESOLUTION = LLT30x430Resolutions.R512;
        private TScannerType _scannerType;
        private uint _exposureTime = 100; // µs
        public uint ExposureTime
        {
            get { return _exposureTime; }
            set { _exposureTime = value; }
        }
        private uint _idleTime = 900; // µs
        public uint IdleTime
        {
            get { return _idleTime; }
            set { _idleTime = value; }
        }

        private bool _connected = false;
        public bool IsConnected { get { return _connected; } }

        // Async/callback-related 
        private readonly AutoResetEvent _profileEvent = new AutoResetEvent(false);
        private byte[]? _asyncProfileBuffer;
        private uint _asyncProfileDataSize;
        private volatile bool _asyncProfileReceived;
        private int _asyncNeededProfiles;
        private int _asyncReceivedProfiles;
        private int _asyncInUse;

        // Map LLT handle -> LLTSensor instance (for callback)
        private static readonly object _cbLock = new object();
        private static readonly Dictionary<uint, LLTSensor> _cbInstances = new Dictionary<uint, LLTSensor>();

        // Single static delegate so GC doesn't collect it
        private static readonly ProfileReceiveMethod _profileCallback;
        #endregion

        public LLTSensor(uint deviceHandle)
        {
            device = deviceHandle;
        }

        static unsafe LLTSensor()
        {
            _profileCallback = new ProfileReceiveMethod(StaticProfileCallback);
        }

        /// <summary>
        /// sets device and then connects.
        /// </summary>
        public bool Connect(uint deviceHandle)
        {
            device = deviceHandle;
            return Connect();
        }

        /// <summary>
        /// Connect to the LLT using existing handle.
        /// </summary>
        public bool Connect(LLT30x430Resolutions resolutions = DEFAULT_RESOLUTION)
        {
            if (device == 0)
                throw new InvalidOperationException("Device handle is 0. CreateLLTDevice or use FindEthernetDevices() first.");

            //uint[] resolutions = new uint[MAX_RESOLUTIONS];

            int ret;

            // Connect
            ret = CLLTI.Connect(device);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("Connect", ret); return false; }

            // Get scanner type (needed for ConvertProfile2Values)
            ret = CLLTI.GetLLTType(device, ref _scannerType);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("GetLLTType", ret); return false; }

            //// Get resolutions and pick the maximum
            //ret = CLLTI.GetResolutions(device, resolutions, resolutions.Length);
            //if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("GetResolutions", ret); return false; }

            //_resolution = resolutions[0];

            //ret = CLLTI.SetResolution(device, _resolution);
            //if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("SetResolution", ret); return false; }

            //Set resolution
            bool resolutionSet =  SetResolution(resolutions);
            if (!resolutionSet) return false;

            // Sensor params 
            uint bufferCount = 20;
            uint mainReflection = 0;
            uint packetSize = 1024;

            ret = CLLTI.SetBufferCount(device, bufferCount);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("SetBufferCount", ret); return false; }

            ret = CLLTI.SetMainReflection(device, mainReflection);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("SetMainReflection", ret); return false; }

            ret = CLLTI.SetPacketSize(device, packetSize);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("SetPacketSize", ret); return false; }

            ret = CLLTI.SetProfileConfig(device, TProfileConfig.PROFILE);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("SetProfileConfig", ret); return false; }

            ret = CLLTI.SetFeature(device, CLLTI.FEATURE_FUNCTION_TRIGGER, CLLTI.TRIG_INTERNAL);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("SetFeature(TRIGGER)", ret); return false; }

            ret = CLLTI.SetFeature(device, CLLTI.FEATURE_FUNCTION_EXPOSURE_TIME, _exposureTime);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("SetFeature(EXPOSURE_TIME)", ret); return false; }

            ret = CLLTI.SetFeature(device, CLLTI.FEATURE_FUNCTION_IDLE_TIME, _idleTime);
            if (ret < CLLTI.GENERAL_FUNCTION_OK) { SetError("SetFeature(IDLE_TIME)", ret); return false; }

            _connected = true;
            ErrorMessage = null;

          
            lock (_cbLock)
            {
                _cbInstances[device] = this;
            }

            // Register callback (once per device)
             ret = CLLTI.RegisterCallback(device, TCallbackType.STD_CALL, _profileCallback, device);
            if (ret < CLLTI.GENERAL_FUNCTION_OK)
            {
                SetError("RegisterCallback", ret);
                _connected = false;
                return false;
            }
            

            return true;
        }


        /// <summary>
        /// Poll ONE profile and return it.
        /// </summary>
        public Profile Poll()
        {
            if (!_connected)
                throw new InvalidOperationException("Sensor not connected. Call Connect() first.");

            int ret;
            uint lostProfiles = 0;

            uint resolution = (uint)_resolution;

            double[] xValues = new double[resolution];
            double[] zValues = new double[resolution];

            byte[] profileBuffer = new byte[resolution * 64];

            // allow parameters to settle
            Thread.Sleep(120);

            // Start continuous profile transmission
            ret = CLLTI.TransferProfiles(device, TTransferProfileType.NORMAL_TRANSFER, 1);
            if (ret < CLLTI.GENERAL_FUNCTION_OK)
            {
                SetError("TransferProfiles(start)", ret);
                throw new Exception(ErrorMessage);
            }

            try
            {
                // Wait for one full profile
                while (true)
                {
                    ret = CLLTI.GetActualProfile(
                        device,
                        profileBuffer,
                        profileBuffer.Length,
                        TProfileConfig.PROFILE,
                        ref lostProfiles);

                    if (ret == CLLTI.ERROR_PROFTRANS_NO_NEW_PROFILE)
                    {
                        Thread.Sleep((int)(_exposureTime + _idleTime) / 100);
                        continue;
                    }

                    if (ret != profileBuffer.Length)
                    {
                        SetError("GetActualProfile", ret);
                        throw new Exception(ErrorMessage);
                    }

                    break;
                }

                // Convert the profile into X/Z arrays
                ret = CLLTI.ConvertProfile2Values(
                    device,
                    profileBuffer,
                    resolution,
                    TProfileConfig.PROFILE,
                    _scannerType,
                    0,   // reflection 0
                    1,   // convert to mm
                    null, null, null,
                    xValues,
                    zValues,
                    null, null);

                if (((ret & CLLTI.CONVERT_X) == 0) || ((ret & CLLTI.CONVERT_Z) == 0))
                {
                    SetError("ConvertProfile2Values", ret);
                    throw new Exception(ErrorMessage);
                }

                Profile profile = new Profile(new List<ScanPoint>((int)resolution));
                for (int i = 0; i < resolution; i++)
                {
                    profile.Add(new ScanPoint
                    {
                        X = xValues[i],
                        Z = zValues[i]
                    });
                }

                ErrorMessage = null;
                return profile;
            }
            finally
            {
                // Always stop transfer
                CLLTI.TransferProfiles(device, TTransferProfileType.NORMAL_TRANSFER, 0);
            }
        }


        public async Task<List<Profile>> AsyncPoll(int profileCount, int timeoutMs = 2000)
        {
            if (!_connected)
                throw new InvalidOperationException("Sensor not connected. Call Connect() first.");

            if (profileCount <= 0)
                throw new ArgumentOutOfRangeException(nameof(profileCount), "profileCount must be > 0.");

            // Ensure we don't run two AsyncPolls at once on this sensor
            if (Interlocked.Exchange(ref _asyncInUse, 1) == 1)
                throw new InvalidOperationException("AsyncPoll is already running.");

            uint resolution = (uint)_resolution;

            try
            {
                // Prepare async buffers/state
                int singleProfileBytes = checked((int)(resolution * 64));
                _asyncProfileBuffer = new byte[singleProfileBytes * profileCount];
                _asyncProfileDataSize = 0;
                _asyncNeededProfiles = profileCount;
                _asyncReceivedProfiles = 0;
                _profileEvent.Reset();

                // Let parameters settle
                await Task.Delay(120);

                // Start continuous profile transmission
                int ret = CLLTI.TransferProfiles(device, TTransferProfileType.NORMAL_TRANSFER, 1);
                if (ret < CLLTI.GENERAL_FUNCTION_OK)
                {
                    SetError("TransferProfiles(start)", ret);
                    throw new Exception(ErrorMessage);
                }

                try
                {
                    // Wait until we have the requested number of profiles or timeout
                    bool signaled = await Task.Run(() => _profileEvent.WaitOne(timeoutMs));

                    if (!signaled || _asyncReceivedProfiles < _asyncNeededProfiles)
                    {
                        SetError("AsyncPoll timeout", CLLTI.ERROR_PROFTRANS_NO_NEW_PROFILE);
                        throw new TimeoutException(
                            $"Timed out waiting for {_asyncNeededProfiles} profiles. Received: {_asyncReceivedProfiles}");
                    }

                    // Sanity-check that profile size is what we expect
                    if (_asyncProfileDataSize != resolution * 64)
                    {
                        SetError("AsyncPoll size mismatch", (int)_asyncProfileDataSize);
                        throw new Exception(
                            $"Profile size mismatch: expected {resolution * 64}, got {_asyncProfileDataSize}");
                    }

                    // Convert each profile slice to X/Z and wrap into Profile objects
                    var profiles = new List<Profile>(_asyncNeededProfiles);
                    var xValues = new double[resolution];
                    var zValues = new double[resolution];

                    var singleBuffer = new byte[singleProfileBytes];

                    for (int p = 0; p < _asyncNeededProfiles; p++)
                    {
                        // Copy this profile’s bytes into a single-profile buffer
                        Buffer.BlockCopy(
                            _asyncProfileBuffer,
                            p * singleProfileBytes,
                            singleBuffer,
                            0,
                            singleProfileBytes);

                        ret = CLLTI.ConvertProfile2Values(
                            device,
                            singleBuffer,
                            resolution,
                            TProfileConfig.PROFILE,
                            _scannerType,
                            0,   // reflection 0
                            1,   // convert to mm
                            null, null, null,
                            xValues,
                            zValues,
                            null, null);

                        if (((ret & CLLTI.CONVERT_X) == 0) || ((ret & CLLTI.CONVERT_Z) == 0))
                        {
                            SetError("ConvertProfile2Values", ret);
                            throw new Exception(ErrorMessage);
                        }

                        var profile = new Profile(new List<ScanPoint>((int)resolution));
                        for (int i = 0; i < resolution; i++)
                        {
                            profile.Add(new ScanPoint
                            {
                                X = xValues[i],
                                Z = zValues[i]
                            });
                        }

                        profiles.Add(profile);
                    }

                    ErrorMessage = null;
                    return profiles;
                }
                finally
                {
                    // Stop transfer
                    CLLTI.TransferProfiles(device, TTransferProfileType.NORMAL_TRANSFER, 0);

                    // Clear async state
                    _asyncProfileBuffer = null;
                    _asyncProfileDataSize = 0;
                    _asyncNeededProfiles = 0;
                    _asyncReceivedProfiles = 0;
                }
            }
            finally
            {
                Interlocked.Exchange(ref _asyncInUse, 0);
            }
        }



        /// <summary>
        /// Static helper: scan for Ethernet LLTs and return a list of (handle, type).
        /// Each handle is a separate LLT device instance (disconnected but usable).
        /// </summary>
        public static List<(uint handle, TScannerType type)> FindEthernetDevices()
        {
            const int MAX_IF = 16;
            var result = new List<(uint handle, TScannerType type)>();

            // Temporary handle just to discover interfaces
            uint discoveryHandle = CLLTI.CreateLLTDevice(TInterfaceType.INTF_TYPE_ETHERNET);
            if (discoveryHandle == 0)
                return result;

            try
            {
                uint[] interfaces = new uint[MAX_IF];
                int count = CLLTI.GetDeviceInterfacesFast(discoveryHandle, interfaces, interfaces.Length);

                if (count <= 0)
                    return result;

                for (int i = 0; i < count && i < interfaces.Length; i++)
                {
                    uint handle = CLLTI.CreateLLTDevice(TInterfaceType.INTF_TYPE_ETHERNET);
                    if (handle == 0)
                        continue;

                    int ret = CLLTI.SetDeviceInterface(handle, interfaces[i], 0);
                    if (ret < CLLTI.GENERAL_FUNCTION_OK)
                    {
                        CLLTI.DelDevice(handle);
                        continue;
                    }

                    ret = CLLTI.Connect(handle);
                    if (ret < CLLTI.GENERAL_FUNCTION_OK)
                    {
                        CLLTI.DelDevice(handle);
                        continue;
                    }

                    TScannerType type = TScannerType.StandardType;
                    ret = CLLTI.GetLLTType(handle, ref type);

                  
                    CLLTI.Disconnect(handle);

                    if (ret < CLLTI.GENERAL_FUNCTION_OK)
                    {
                        CLLTI.DelDevice(handle);
                        continue;
                    }

                    result.Add((handle, type));
                }
            }
            finally
            {
                CLLTI.DelDevice(discoveryHandle);
            }

            return result;
        }

        /// <summary>
        /// Helper to populate ErrorMessage using TranslateErrorValue.
        /// </summary>
        private void SetError(string context, int errorCode)
        {
            try
            {
                byte[] buffer = new byte[256];
                int ret = CLLTI.TranslateErrorValue(device, errorCode, buffer, buffer.Length);

                if (ret >= CLLTI.GENERAL_FUNCTION_OK)
                {
                    string text = Encoding.ASCII
                        .GetString(buffer, 0, buffer.Length)
                        .TrimEnd('\0', '\r', '\n');

                    ErrorMessage = $"{context}: {text} (code {errorCode})";
                }
                else
                {
                    ErrorMessage = $"{context}: error code {errorCode}";
                }
            }
            catch
            {
                ErrorMessage = $"{context}: error code {errorCode}";
            }
        }

        public bool SetResolution(LLT30x430Resolutions resolution)
        {
            if (!_connected)
                throw new InvalidOperationException("Sensor not connected. Call Connect() first.");
            int ret = CLLTI.SetResolution(device, (uint)resolution);
            if (ret < CLLTI.GENERAL_FUNCTION_OK)
            {
                SetError("SetResolution", ret);
                return false;
            }
            _resolution = resolution;
            return true;
        }

        unsafe private static void StaticProfileCallback(byte* data, uint size, uint userData)
        {
            LLTSensor sensor = null;
            lock (_cbLock)
            {
                _cbInstances.TryGetValue(userData, out sensor);
            }

            sensor?.OnProfileCallback(data, size);
        }

        unsafe private void OnProfileCallback(byte* data, uint size)
        {
            if (size == 0)
                return;

            // If we aren't currently waiting, or buffer not prepared, ignore
            if (_asyncProfileBuffer == null || _asyncNeededProfiles <= 0)
                return;

            // If we already got everything we need, ignore extra profiles
            if (_asyncReceivedProfiles >= _asyncNeededProfiles)
                return;

            // First profile: remember its size
            if (_asyncProfileDataSize == 0)
            {
                _asyncProfileDataSize = size;
            }
            else if (size != _asyncProfileDataSize)
            {
                // Inconsistent packet size – ignore or log; here we bail out
                return;
            }

            int singleSize = (int)_asyncProfileDataSize;
            int offset = _asyncReceivedProfiles * singleSize;

            // Safety: make sure we don't overflow the buffer
            if (offset + singleSize > _asyncProfileBuffer.Length)
                return;

            // Copy unmanaged data → managed buffer at the correct offset
            Marshal.Copy((IntPtr)data, _asyncProfileBuffer, offset, singleSize);

            _asyncReceivedProfiles++;

            // If we’ve collected everything, wake up AsyncPoll
            if (_asyncReceivedProfiles >= _asyncNeededProfiles)
            {
                _profileEvent.Set();
            }
        }



        public void Disconnect()
        {
            if (!_connected)
                return;

            CLLTI.Disconnect(device);

            lock (_cbLock)
            {
                _cbInstances.Remove(device);
            }

            _connected = false;
        }
    }
}
