using System;
using TwinCAT.Ads;

namespace LLT
{
    internal class ADS : IDisposable
    {
        public string AMSNetID;
        public StateInfo ConnectionState;

        private AdsClient _client;
        private bool _connected;

        public ADS(string AMSNetID = "", int Port = 851)
        {
            this.AMSNetID = AMSNetID;
            _client = new AdsClient();

            if (!string.IsNullOrWhiteSpace(AMSNetID))
                Connect(AMSNetID, Port);
        }

        public bool Connect(string AMSNetID = "", int Port = 851)
        {
            // If caller passes empty, use stored AMSNetID
            AMSNetID = string.IsNullOrWhiteSpace(AMSNetID) ? this.AMSNetID : AMSNetID;
            this.AMSNetID = AMSNetID;

            try
            {
                if (_connected)
                    Disconnect();

                _client.Connect(AMSNetID, Port);
                ConnectionState = _client.ReadState();
                _connected = (ConnectionState.AdsState == AdsState.Run);
                return _connected;
            }
            catch
            {
                _connected = false;
                return false;
            }
        }

        public void Disconnect()
        {
            try
            {
                _client?.Disconnect();
            }
            catch { /* ignore */ }
            _connected = false;
        }

        /// <summary>
        /// Writes a PLC symbol by name. Supports scalars + arrays.
        /// For arrays, pass the array instance (e.g., double[]).
        /// </summary>
        public bool WriteVal(string varName, object val)
        {
            if (!_connected) return false;
            if (string.IsNullOrWhiteSpace(varName)) return false;
            if (val == null) return false;

            int handle = 0;

            try
            {
                handle = _client.CreateVariableHandle(varName);

                // WriteAny handles most primitives and arrays as long as the .NET type matches PLC type.
                // Example: PLC LREAL[] <-> double[]
                _client.WriteAny(handle, val);

                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                if (handle != 0)
                {
                    try { _client.DeleteVariableHandle(handle); } catch { /* ignore */ }
                }
            }
        }

        /// <summary>
        /// Strongly-typed write (recommended).
        /// </summary>
        public bool WriteVal<T>(string varName, T val)
        {
            return WriteVal(varName, (object)val);
        }

        /// <summary>
        /// Reads a PLC symbol by name into type T.
        /// For arrays, use T like double[].
        /// </summary>
        public bool ReadVal<T>(string varName, out T value)
        {
            value = default;
            if (!_connected) return false;
            if (string.IsNullOrWhiteSpace(varName)) return false;

            int handle = 0;

            try
            {
                handle = _client.CreateVariableHandle(varName);
                object obj = _client.ReadAny(handle, typeof(T));
                value = (T)obj;
                return true;
            }
            catch
            {
                value = default;
                return false;
            }
            finally
            {
                if (handle != 0)
                {
                    try { _client.DeleteVariableHandle(handle); } catch { /* ignore */ }
                }
            }
        }

        public void Dispose()
        {
            Disconnect();
            _client?.Dispose();
            _client = null;
        }
    }
}