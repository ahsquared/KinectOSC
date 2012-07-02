
using System.Collections.Generic;
using System.Linq;

namespace KinectOSC
{
    //using Bespoke.Common.Osc;
    using Ventuz.OSC;
    using System;
    using System.Globalization;
    using System.IO;
    using System.Net;
    using System.Threading;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap colorBitmap;

        /// <summary>
        /// Intermediate storage for the color data received from the camera
        /// </summary>
        private byte[] colorPixels;

        /// <summary>
        /// Store the Draw Color checkbox value for quick access
        /// </summary>
        private bool drawColor = true;

        /// <summary>
        /// Store the OSC On checkbox value for quick access
        /// </summary>
        private bool oscOn = true;

        /// <summary>
        /// Store the Show OSC Data checkbox value for quick access
        /// </summary>
        private bool showOscData = false;

        /// <summary>
        /// If this is true the OSC data will be sent as a single string; else it will be sent as bundles of floats (x, y, z) for each joint
        /// </summary>
        private bool sendOscAsString = false;

        /// <summary>
        /// If this is true the skeleton will be drawn on screen
        /// </summary>
        private bool showSkeleton = false;

        /// <summary>
        /// If this is true then only the skeleton nearest the kinect will be tracked
        /// </summary>
        private bool trackNearestOnly = false;

        /// <summary>
        /// If this is true then the positions of the joints will be sent as percentages of the width and height
        /// </summary>
        private bool sendPositionsAsPercentage = true;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        // variables for OSC server using Bespoke - doesn't seem to work properly
        //private static readonly TransportType TransportType = TransportType.Udp;
        //private static readonly IPAddress MulticastAddress = IPAddress.Parse("127.0.0.1");
        //private static readonly int Port = 3001;
        //private static readonly IPEndPoint Destination = new IPEndPoint(MulticastAddress, Port);
        //private static OscBundle sBundle;

        // OSC 
        private static UdpWriter oscWriter;
        private static string[] oscArgs = new string[2];

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }


        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right)) {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit
            foreach (var potentialSensor in KinectSensor.KinectSensors) {
                if (potentialSensor.Status == KinectStatus.Connected) {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor) {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Turn on the color stream to receive color frames
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

                // Allocate space to put the pixels we'll receive
                this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                this.ColorImage.Source = this.colorBitmap;

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.ColorFrameReady += this.SensorColorFrameReady;

                // Start the sensor!
                try {
                    this.sensor.Start();
                }
                catch (IOException) {
                    this.sensor = null;
                }
            }

            if (null == this.sensor) {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }

            // Setup osc sender
            oscArgs[0] = "127.0.0.1";
            oscArgs[1] = OscPort.Text;
            oscWriter = new UdpWriter(oscArgs[0], Convert.ToInt32(oscArgs[1]));


        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor) {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Draw the Color Frame to screen
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            if (drawColor)
            {
                using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
                {
                    if (colorFrame != null)
                    {
                        // Copy the pixel data from the image to a temporary array
                        colorFrame.CopyPixelDataTo(this.colorPixels);

                        // Write the pixel data into our bitmap
                        this.colorBitmap.WritePixels(
                            new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                            this.colorPixels,
                            this.colorBitmap.PixelWidth*sizeof (int),
                            0);
                    }
                }
            }
        }

        /// <summary>
        /// Set kinect to track the closest skeleton
        /// </summary>
        private void TrackClosestSkeleton(IEnumerable<Skeleton> skeletonData)
        {
            if (this.sensor != null && this.sensor.SkeletonStream != null) {
                if (!this.sensor.SkeletonStream.AppChoosesSkeletons) {
                    this.sensor.SkeletonStream.AppChoosesSkeletons = true; // Ensure app chooses skeletons is set
                }

                float closestDistance = 10000f; // Start with a far enough distance
                int closestID = 0;

                foreach (Skeleton skeleton in skeletonData.Where(s => s.TrackingState != SkeletonTrackingState.NotTracked)) {
                    if (skeleton.Position.Z < closestDistance) {
                        closestID = skeleton.TrackingId;
                        closestDistance = skeleton.Position.Z;
                    }
                }

                if (closestID > 0) {
                    this.sensor.SkeletonStream.ChooseSkeletons(closestID); // Track this skeleton
                }
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame()) {
                if (skeletonFrame != null) {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            if (trackNearestOnly)
            {
                TrackClosestSkeleton(skeletons);
            }

            using (DrawingContext dc = this.drawingGroup.Open()) {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0) {
                    var counter = 0;
                    foreach (Skeleton skel in skeletons) {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked) {
                            if (showSkeleton)
                            {
                                this.DrawBonesAndJoints(skel, dc);
                            }
                            if (oscOn)
                            {
                                
                                var elements = new List<OscElement>();
                                var args = "";
                                var jointNumber = 0;
                                var oscText = "";
                                if (sendOscAsString) {
                                    // bundle all joints as a string for parsing at client end
                                    // "Joint name|Joint number:.56|.45|1.4, ... more joints"
                                    foreach (Joint joint in skel.Joints) {
                                        jointNumber++;
                                        var jointName = joint.JointType.ToString();
                                        Point jointPosition = SkeletonPointToScreen(joint.Position);
                                        if (sendPositionsAsPercentage) {
                                            jointPosition.X = Math.Round(jointPosition.X / 640, 3);
                                            jointPosition.Y = Math.Round(jointPosition.Y / 480, 3);
                                            // concatenate all joints as one big string to send as OSC
                                            args += jointName + "|" + jointNumber + ":" + jointPosition.X + "|" +
                                                    jointPosition.Y + "|" + Math.Round(joint.Position.Z,3) + ",";
                                            if (showOscData) {
                                                oscText += GenerateOscDataDump(counter, jointName, joint.Position, jointPosition);
                                            }
                                        }
                                        else
                                        {
                                            // concatenate all joints as one big string to send as OSC
                                            args += jointName + "|" + jointNumber + ":" + Math.Round(joint.Position.X,4) + "|" +
                                                    Math.Round(joint.Position.Y,4) + "|" + Math.Round(joint.Position.Z,4) + ",";

                                            if (showOscData) {
                                                oscText += GenerateOscDataDump(counter, jointName, joint.Position);
                                            }
                                        }

                                    }
                                    elements.Add(new OscElement("/skeleton" + counter, args));
                                    oscWriter.Send(new OscBundle(DateTime.Now, elements.ToArray()));
                                }
                                else
                                {
                                    // joints bundled individually as 3 floats (x, y, z)
                                    foreach (Joint joint in skel.Joints) {
                                        var jointName = joint.JointType.ToString();
                                        var jointElement = new List<OscElement>();
                                        Point jointPosition = SkeletonPointToScreen(joint.Position);
                                        if (sendPositionsAsPercentage)
                                        {
                                            jointPosition.X = Math.Round(jointPosition.X / 640, 3);
                                            jointPosition.Y = Math.Round(jointPosition.Y / 480, 3);
                                            jointElement.Add(new OscElement("/skeleton" + counter + "/" + jointName,
                                                                            (float) jointPosition.X,
                                                                            (float) jointPosition.Y, Math.Round(joint.Position.Z,3)));
                                            oscWriter.Send(new OscBundle(DateTime.Now, jointElement.ToArray()));
                                            if (showOscData)
                                            {
                                                oscText += GenerateOscDataDump(counter, jointName, joint.Position, jointPosition);
                                            }
                                        }
                                        else {
                                            jointElement.Add(new OscElement("/skeleton" + counter + "/" + jointName, Math.Round(joint.Position.X,4), Math.Round(joint.Position.Y,4), Math.Round(joint.Position.Z,4)));
                                            oscWriter.Send(new OscBundle(DateTime.Now, jointElement.ToArray()));
                                            if (showOscData) {
                                                oscText += GenerateOscDataDump(counter, jointName, joint.Position);
                                            }
                                        }

                                    }
                                    
                                }

                                if (showOscData)
                                {
                                    oscViewer.Text = oscText;
                                    showOscData = false;
                                    //checkBoxShowOscData.IsChecked = false;
                                }
                                counter++;
                            }
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly) {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        private String GenerateOscDataDump(int counter, string jointName, SkeletonPoint jointPoint)
        {
            var dataDump = "";
            dataDump += "/skeleton" + counter + "/" + jointName + "/x" + Math.Round(jointPoint.X, 4) + "\n";
            dataDump += "/skeleton" + counter + "/" + jointName + "/y" + Math.Round(jointPoint.Y, 4) + "\n";
            dataDump += "/skeleton" + counter + "/" + jointName + "/z" + Math.Round(jointPoint.Z, 4) + "\n";
            return dataDump;
        }

        private String GenerateOscDataDump(int counter, string jointName, SkeletonPoint jointPoint, Point jointPosition)
        {
            var dataDump = "";
            dataDump += "/skeleton" + counter + "/" + jointName + "/x" + Math.Round(jointPosition.X, 3) + "\n";
            dataDump += "/skeleton" + counter + "/" + jointName + "/y" + Math.Round(jointPosition.Y, 3) + "\n";
            dataDump += "/skeleton" + counter + "/" + jointName + "/z" + Math.Round(jointPoint.Z, 3) + "\n";
            return dataDump;
        }


        public static long GetTimestamp()
        {
            long ticks = DateTime.UtcNow.Ticks - DateTime.Parse("01/01/1970 00:00:00").Ticks;
            ticks /= 10000;
            return ticks;
        }


        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints
            foreach (Joint joint in skeleton.Joints) {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked) {
                    drawBrush = this.trackedJointBrush;
                }
                else if (joint.TrackingState == JointTrackingState.Inferred) {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null) {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.MapSkeletonPointToDepth(
                                                                             skelpoint,
                                                                             DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked) {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred) {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked) {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor) {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault()) {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }
        /// <summary>
        /// Handles the checking or unchecking of the draw color checkbox
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxDrawColorChanged(object sender, RoutedEventArgs e)
        {
            drawColor = this.checkBoxDrawColor.IsChecked.GetValueOrDefault();
            ResizeWindow();
        }

        private void CheckBoxOscOnChanged(object sender, RoutedEventArgs e)
        {
            oscOn = this.checkBoxOscOn.IsChecked.GetValueOrDefault();
        }
        private void CheckBoxShowOscDataChanged(object sender, RoutedEventArgs e)
        {
            showOscData = this.checkBoxShowOscData.IsChecked.GetValueOrDefault();
            if (showOscData)
            {
                oscViewer.Visibility = Visibility.Visible;
                CloseOscViewer.Visibility = Visibility.Visible;
            }
            else
            {
                oscViewer.Visibility = Visibility.Collapsed;
                CloseOscViewer.Visibility = Visibility.Collapsed;                
            }
        }
        private void CheckBoxShowSkeletonChanged(object sender, RoutedEventArgs e)
        {
            showSkeleton = this.checkBoxShowSkeleton.IsChecked.GetValueOrDefault();
            ResizeWindow();
        }

        private void ResizeWindow()
        {
            if (!drawColor && !showSkeleton) {
                this.Height = 230;
            }
            else
            {
                this.Height = 755;
            }
        }
        private void CheckBoxTrackNearestOnlyChanged(object sender, RoutedEventArgs e)
        {
            trackNearestOnly = this.checkBoxTrackNearestOnly.IsChecked.GetValueOrDefault();
        }

        private void CheckBoxSendPositionAsPercentageChanged(object sender, RoutedEventArgs e)
        {
            sendPositionsAsPercentage = this.checkBoxSendPositionAsPercentage.IsChecked.GetValueOrDefault();
        }

        
        private void CheckBoxSendAsStringChanged(object sender, RoutedEventArgs e)
        {
            sendOscAsString = this.checkBoxSendAsString.IsChecked.GetValueOrDefault();
        }
        private void ChangePortClicked(object sender, RoutedEventArgs e)
        {
            oscArgs[1] = OscPort.Text;
            oscWriter = new UdpWriter(oscArgs[0], Convert.ToInt32(oscArgs[1]));
        }

        private void CloseOscViewerClicked(object sender, RoutedEventArgs e)
        {
            oscViewer.Visibility = Visibility.Collapsed;
            CloseOscViewer.Visibility = Visibility.Collapsed;
            checkBoxShowOscData.IsChecked = false;
        }
    }
}