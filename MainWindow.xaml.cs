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
using HelixToolkit.Wpf;
using Microsoft.Win32;
using System.Windows.Media.Media3D;
using System.Numerics;
using static LLT.LLTSensor;

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
            ProfilePlot.Model = _plotModel;

            _rateStopwatch.Start();

            StartPolling();
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

        #region Profile Methods

        private void StartButton_Click(object sender, RoutedEventArgs e)
        {
            if (_sensor == null)
            {
                MessageBox.Show("Connect to a sensor first.");
                return;
            }

            if (_timer != null)
                return;

            StartPolling();
        }

        private void StopButton_Click(object sender, RoutedEventArgs e)
        {
            StopPolling();
        }
        private void StartPolling()
        {
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
            //if (_sensor == null)
            //    return;

            try
            {

                //var profile = _sensor.Poll();
                var profile = GenerateRandomProfile();
                _profileSeries.Points.Clear();
                foreach (var p in profile.ToArray())
                {
                    _profileSeries.Points.Add(new DataPoint(p.X, p.Z));
                }

                _plotModel.InvalidatePlot(true);

                
                _profilesThisSecond++;
                if (_rateStopwatch.ElapsedMilliseconds >= 1000)
                {
                    FrequencyText.Text = $"Profiles/s: {_profilesThisSecond}";
                    _profilesThisSecond = 0;
                    _rateStopwatch.Restart();
                }

                //if (!string.IsNullOrEmpty(_sensor.ErrorMessage))
                //    LastErrorText.Text = "Last error: " + _sensor.ErrorMessage;
            }
            catch (Exception ex)
            {
                LastErrorText.Text = "Last error: " + ex.Message;
            }
        }

        public static Profile GenerateRandomProfile(double spanMm = 430.0)
        {
            int n = 2048;
            Random _rnd = new Random();

            // X from -span/2 .. +span/2
            double xMin = -spanMm / 2.0;
            double xMax = +spanMm / 2.0;
            double dx = (xMax - xMin) / (n - 1);

            var points = new List<ScanPoint>(n);

            // Create 2–4 random "bumps" along X
            int peakCount = _rnd.Next(2, 5);
            var peakCenters = new double[peakCount];
            var peakAmps = new double[peakCount];
            var peakWidths = new double[peakCount];

            for (int i = 0; i < peakCount; i++)
            {
                // random center along span
                peakCenters[i] = xMin + _rnd.NextDouble() * (xMax - xMin);
                // amplitude between 5 and 25 mm
                peakAmps[i] = 5.0 + _rnd.NextDouble() * 20.0;
                // width (sigma) between 10 and 60 mm
                peakWidths[i] = 10.0 + _rnd.NextDouble() * 50.0;
            }

            // Base Z level so everything is positive-ish
            double baseZ = 100.0;

            for (int i = 0; i < n; i++)
            {
                double x = xMin + i * dx;

                // Start with base level
                double z = baseZ;

                // Add Gaussian-ish bumps
                for (int p = 0; p < peakCount; p++)
                {
                    double dxp = x - peakCenters[p];
                    double sigma = peakWidths[p];
                    double gauss = Math.Exp(-(dxp * dxp) / (2.0 * sigma * sigma));
                    z += peakAmps[p] * gauss;
                }

                // Add small noise (±1 mm)
                double noise = (_rnd.NextDouble() - 0.5) * 2.0 * 1.0;
                z += noise;

                points.Add(new ScanPoint
                {
                    X = x,
                    Z = z
                });
            }

            return new Profile(points);
        }
        #endregion

        #region PointCloud Methods
        private void LoadStlButton_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog
            {
                Title = "Select STL file",
                Filter = "STL files (*.stl)|*.stl|All files (*.*)|*.*",
                CheckFileExists = true,
                Multiselect = false
            };

            
            //        Filter =
            //"3D Models|*.stl;*.obj;*.3ds;*.dae;*.ply;*.fbx|" +
            //"STL (*.stl)|*.stl|" +
            //"OBJ (*.obj)|*.obj|" +
            //"All Files (*.*)|*.*"

            if (dlg.ShowDialog(this) != true)
                return;

            try
            {
                var importer = new ModelImporter();

                // Load STL as a 3D model
                Model3D model = importer.Load(dlg.FileName);


                // Compute bounding box width (for side-by-side spacing)
                var bounds = model.Bounds;
                double width = bounds.SizeX;
                if (width <= 0) width = 1.0; // safety fallback

                // Add a bit of gap between them
                double offset = width * 1.2;

                //Clear Displayed
                MeshModel.Content = null;
                PointCloudPoints.Points.Clear();
                LblPointCloudCnt.Content = "-";

                //Load Mesh
                Loadmesh(model, offset);

                //Load Point Cloud
                LoadPointCloud(model, offset);

                // One camera, both objects in frame
                Viewport.ZoomExtents();
            }
            catch (Exception ex)
            {
                MessageBox.Show(this,
                    "Failed to load STL file:\n" + ex.Message,
                    "Error",
                    MessageBoxButton.OK,
                    MessageBoxImage.Error);
            }
        }


        private void Loadmesh(Model3D model, double offset = 1)
        {

            // Mesh on the LEFT
            MeshModel.Content = model;
            MeshModel.Transform = new TranslateTransform3D(-offset / 2.0, 0, 0);
        }

        private void LoadPointCloud(Model3D model, double offset = 1)
        {

            // Extract vertices into a point cloud
            var points = new Point3DCollection();

            void CollectPoints(Model3D m)
            {
                if (m is Model3DGroup group)
                {
                    foreach (var child in group.Children)
                        CollectPoints(child);
                }
                else if (m is GeometryModel3D geom &&
                         geom.Geometry is MeshGeometry3D mesh)
                {
                    foreach (var p in mesh.Positions)
                        points.Add(p);
                }
            }

            CollectPoints(model);

            // Point cloud on the RIGHT
            var downsampledPoints = PointCloudProcessing.VoxelDownSample(points, 0.5);
            PointCloudPoints.Points = downsampledPoints;
            PointCloudPoints.Transform = new TranslateTransform3D(+offset / 2.0, 0, 0);

            LblPointCloudCnt.Content = downsampledPoints.Count;
            Debug.Print($"Loaded {downsampledPoints.Count} points from STL.");
        }

        private void BtnPoject_Click(object sender, RoutedEventArgs e)
        {

            var cloud = PointCloudPoints.Points;
            var p0 = cloud[0];
            var p1 = cloud[1];
            var p2 = cloud[2];

            
            //var projected3D = PointCloudProcessing.ProjectPointsToPlane3D(cloud, p0, p1, p2);
            var projected3D = PointCloudProcessing.ProjectToFace3D(cloud, ViewFace.Back);
            ProjectedPoints.Points = null;
            ProjectedPoints.Points = projected3D;
            ////plot projection
            //var projected = PointCloudProcessing.ProjectModelToFacePlane(cloud, p0, p1, p2);
            //_profileSeries.Points.Clear();
            //_profileSeries.Title = "Projected Profile";
            //_profileSeries.Points.AddRange(projected);
            //_plotModel.InvalidatePlot(true);
        }
        #endregion


    }
}