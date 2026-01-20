public bool ReadVal(string varName, Type type, out object value)
{
    value = null;
    if (!_connected) return false;

    int handle = 0;
    try
    {
        handle = _client.CreateVariableHandle(varName);
        value = _client.ReadAny(handle, type);
        return true;
    }
    finally
    {
        if (handle != 0) _client.DeleteVariableHandle(handle);
    }
}