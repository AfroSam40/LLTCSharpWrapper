using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;

namespace ADSRouteHealthChecker
{
    public class Program
    {
        public static void Main(string[] args)
        {
            Host.CreateDefaultBuilder(args)
                .UseWindowsService()
                .ConfigureServices((context, services) =>
                {
                    services.AddHostedService<Worker>();
                })
                .Build()
                .Run();
        }
    }
}