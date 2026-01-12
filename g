using System;
using TwinCAT.Ads;

class Program
{
    static void Main()
    {
        // PLC AMS Net ID example: "192.168.1.10.1.1"
        // PLC Runtime 1 ADS Port: 851
        string plcAmsNetId = "192.168.1.10.1.1";
        int plcPort = 851;

        using var client = new AdsClient();
        client.Connect(plcAmsNetId, plcPort);

        // Example PLC symbol name (change to your actual variable)
        string symbolName = "MAIN.bMyBool";

        // Read
        using (var handle = client.CreateVariableHandle(symbolName))
        {
            bool value = (bool)client.ReadAny(handle, typeof(bool));
            Console.WriteLine($"Read {symbolName} = {value}");

            // Write example
            client.WriteAny(handle, !value);
            Console.WriteLine($"Wrote {symbolName} = {!value}");
        }
    }
}