using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;

namespace ADSRouteHealthChecker
{
    public class Worker : BackgroundService
    {
        private readonly ILogger<Worker> _logger;
        private const string PLC_IP = "10.60.68.105.1.1";

        public Worker(ILogger<Worker> logger)
        {
            _logger = logger;
        }

        protected override async Task ExecuteAsync(CancellationToken stoppingToken)
        {
            _logger.LogInformation("Worker starting");

            await Task.Yield(); // let Windows service startup complete first

            while (!stoppingToken.IsCancellationRequested)
            {
                try
                {
                    _logger.LogInformation("Worker running at: {time}", DateTimeOffset.Now);

                    await Task.Run(() =>
                    {
                        using var plc = new ADS();
                        plc.Connect(PLC_IP);

                        var adsState = plc.ConnectionState.AdsState;
                        _logger.LogInformation("ADS state = {adsState}", adsState);
                    }, stoppingToken);
                }
                catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
                {
                    break;
                }
                catch (Exception ex)
                {
                    _logger.LogWarning(ex, "ADS error while probing PLC.");
                }

                await Task.Delay(TimeSpan.FromSeconds(15), stoppingToken);
            }

            _logger.LogInformation("Worker stopping");
        }
    }
}