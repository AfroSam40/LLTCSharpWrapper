using OxyPlot.Series;
using OxyPlot;
using System.Diagnostics;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using OxyPlot.Axes;
using MEScanControl;

namespace LLT
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {

        private LLTSensor? _sensor;
        private DispatcherTimer? _timer;

        private PlotModel _plotModel;
        private LineSeries _profileSeries;

        private readonly Stopwatch _rateStopwatch = new Stopwatch();
        private int _profilesThisSecond = 0;

        public MainWindow()
        {
            InitializeComponent();

            // Set up OxyPlot
            _plotModel = new PlotModel { Title = "Profile" };

            _plotModel.Axes.Add(new LinearAxis
            {
                Position = AxisPosition.Bottom,
                Title = "X",
                IsZoomEnabled = true,
                IsPanEnabled = true
            });

            _plotModel.Axes.Add(new LinearAxis
            {
                Position = AxisPosition.Left,
                Title = "Z",
                IsZoomEnabled = true,
                IsPanEnabled = true
            });

            _profileSeries = new LineSeries
            {
                StrokeThickness = 1,
                LineStyle = LineStyle.Solid
            };

            _plotModel.Series.Add(_profileSeries);
            ProfilePlot.DataContext = _plotModel;

            _rateStopwatch.Start();
        }

        // Simple device info for the combo box
        private class DeviceInfo
        {
            public uint Handle { get; set; }
            public TScannerType ScannerType { get; set; }
            public string Display => $"{Handle} - {ScannerType}";
        }

        private void FindButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                (sender as Button).IsEnabled = false;
                var devices = LLTSensor.FindEthernetDevices(); 

                var list = new List<DeviceInfo>();
                foreach (var d in devices)
                {
                    list.Add(new DeviceInfo
                    {
                        Handle = d.handle,
                        ScannerType = d.type
                    });
                }

                DeviceCombo.ItemsSource = list;

                if (list.Count > 0)
                    DeviceCombo.SelectedIndex = 0;

                StatusText.Text = list.Count > 0 ? "Devices found" : "No devices";
                StatusText.Foreground = list.Count > 0 ? System.Windows.Media.Brushes.DarkGreen
                                                       : System.Windows.Media.Brushes.DarkRed;
            }
            catch (Exception ex)
            {
                StatusText.Text = "Find failed";
                StatusText.Foreground = System.Windows.Media.Brushes.DarkRed;
                LastErrorText.Text = "Last error: " + ex.Message;
            }
            finally
            {
                (sender as Button)!.IsEnabled = true;
            }
        }

        private void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (_sensor == null)
            {
                // Connect
                if (DeviceCombo.SelectedItem is not DeviceInfo dev)
                {
                    MessageBox.Show("Select a device first.");
                    return;
                }

                _sensor = new LLTSensor(dev.Handle);

                try
                {
                    if (!_sensor.Connect())
                    {
                        MessageBox.Show("Connect failed: " + (_sensor.ErrorMessage ?? "Unknown error"));
                        _sensor = null;
                        return;
                    }

                    StatusText.Text = "Connected";
                    StatusText.Foreground = System.Windows.Media.Brushes.DarkGreen;
                    LastErrorText.Text = "Last error: -";
                    ConnectButton.Content = "Disconnect";
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Connect exception: " + ex.Message);
                    _sensor = null;
                }
            }
            else
            {
                // Disconnect
                try
                {
                    StopPolling();
                    _sensor.Disconnect();
                }
                catch
                {
                    // ignore
                }
                finally
                {
                    _sensor = null;
                    StatusText.Text = "Disconnected";
                    StatusText.Foreground = System.Windows.Media.Brushes.DarkRed;
                    ConnectButton.Content = "Connect";
                }
            }
        }

        private void StartButton_Click(object sender, RoutedEventArgs e)
        {
            if (_sensor == null)
            {
                MessageBox.Show("Connect to a sensor first.");
                return;
            }

            if (_timer != null)
                return;

            if (!int.TryParse(PollIntervalText.Text, out int intervalMs) || intervalMs <= 0)
                intervalMs = 50;

            _timer = new DispatcherTimer(DispatcherPriority.Background)
            {
                Interval = TimeSpan.FromMilliseconds(intervalMs)
            };
            _timer.Tick += Timer_Tick;
            _timer.Start();

            StartButton.IsEnabled = false;
            StopButton.IsEnabled = true;
        }

        private void StopButton_Click(object sender, RoutedEventArgs e)
        {
            StopPolling();
        }

        private void StopPolling()
        {
            if (_timer != null)
            {
                _timer.Stop();
                _timer.Tick -= Timer_Tick;
                _timer = null;
            }

            StartButton.IsEnabled = true;
            StopButton.IsEnabled = false;
        }

        private void Timer_Tick(object? sender, EventArgs e)
        {
            if (_sensor == null)
                return;

            try
            {
                // Synchronous poll of ONE profile.
                // If you have AsyncPollSingle(), you can use that with async/await instead.
                var profile = _sensor.Poll();

                _profileSeries.Points.Clear();
                foreach (var p in profile.ToArray())
                {
                    _profileSeries.Points.Add(new DataPoint(p.X, p.Z));
                }

                _plotModel.InvalidatePlot(true);

                // Simple profiles/s estimation
                _profilesThisSecond++;
                if (_rateStopwatch.ElapsedMilliseconds >= 1000)
                {
                    FrequencyText.Text = $"Profiles/s: {_profilesThisSecond}";
                    _profilesThisSecond = 0;
                    _rateStopwatch.Restart();
                }

                if (!string.IsNullOrEmpty(_sensor.ErrorMessage))
                    LastErrorText.Text = "Last error: " + _sensor.ErrorMessage;
            }
            catch (Exception ex)
            {
                LastErrorText.Text = "Last error: " + ex.Message;
            }
        }
    }
}