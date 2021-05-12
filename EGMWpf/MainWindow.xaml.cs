using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Net;
using Valve.VR;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Net.Sockets;
using ABB.Robotics.Controllers;
using System.Diagnostics;
using System.Net.NetworkInformation;
using System.Threading;
using ABB.Robotics.Controllers.Discovery;

namespace EGMWpf
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {

        Thread updateThread;
        CVRSystem headset;
        EGM_6_10.UDPUC_RW6_10 EGMCommunication;
        ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal grip;
        Controller ActiveController;

        bool startRobot = false;
        BoundProperties BoundProperties = new BoundProperties();
        public MainWindow()
        {
            InitializeComponent();

            NetworkScanner scanner = new ABB.Robotics.Controllers.Discovery.NetworkScanner();
            NetworkScanner.AddRemoteController("127.0.0.1");
            scanner.Scan();

            ControllerInfoCollection controllers = scanner.Controllers;
            DataContext = BoundProperties;
            BoundProperties.FoundControllers = controllers;
            VRInitialize();
            EGMCommunication = new EGM_6_10.UDPUC_RW6_10(6510, EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion);
            EGMCommunication.Start();
        }
        private void VRInitialize()
        {
            EVRInitError error = new EVRInitError();
            headset = OpenVR.Init(ref error, EVRApplicationType.VRApplication_Background);
            BoundProperties.VRInitializeError = error;
            //BoundProperties.VRControllerPosition = new EGMVector3(25, 5, 1231);
            //BoundProperties.VRControllerRotation = new EGMVector3(30, 40, 60);
            //BoundProperties.VRControllerPosition2 = new EGMVector3(23423, 2, 1231);
            //BoundProperties.VRControllerRotation2 = new EGMVector3(303, 403, 603);
            
        }
        private void VRThread()
        {
            uint controller1Index = 1;
            uint controller2Index = 2;
            Quaternion Quat1;
            Quaternion Quat2;
            Matrix4x4 Mat4x4;
            VRControllerState_t BeginState = new VRControllerState_t();
            VRControllerState_t BeginState2 = new VRControllerState_t();
            TrackedDevicePose_t StartPose = new TrackedDevicePose_t();
            TrackedDevicePose_t StartPose2 = new TrackedDevicePose_t();
            headset.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, controller1Index, ref BeginState, BeginState.unPacketNum, ref StartPose);
            headset.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, controller2Index, ref BeginState2, BeginState.unPacketNum, ref StartPose2);
            Vector3 startPos = new Vector3(StartPose.mDeviceToAbsoluteTracking.m3, StartPose.mDeviceToAbsoluteTracking.m7, StartPose.mDeviceToAbsoluteTracking.m11);
            startPos = Vector3.Transform(startPos, new Quaternion(.707106f, 0, 0, .707106f));
            startPos = Vector3.Transform(startPos, new Quaternion(0, 0, .707106f, .707106f));
            
            do
            {

                //TrackedDevicePose_t[] t = new TrackedDevicePose_t[6];
                //headset.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0, t);
                VRControllerState_t controllerState = new VRControllerState_t();
                VRControllerState_t controllerState2 = new VRControllerState_t();
                TrackedDevicePose_t position = new TrackedDevicePose_t();
                TrackedDevicePose_t position2 = new TrackedDevicePose_t();

                headset.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, controller1Index, ref controllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)), ref position);
                headset.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, controller2Index, ref controllerState2, (uint)Marshal.SizeOf(typeof(VRControllerState_t)), ref position2);

                Mat4x4 = new System.Numerics.Matrix4x4(
                    position.mDeviceToAbsoluteTracking.m0, position.mDeviceToAbsoluteTracking.m4, position.mDeviceToAbsoluteTracking.m8, 0,
                    position.mDeviceToAbsoluteTracking.m1, position.mDeviceToAbsoluteTracking.m5, position.mDeviceToAbsoluteTracking.m9, 0,
                    position.mDeviceToAbsoluteTracking.m2, position.mDeviceToAbsoluteTracking.m6, position.mDeviceToAbsoluteTracking.m10, 0,
                    position.mDeviceToAbsoluteTracking.m3, position.mDeviceToAbsoluteTracking.m7, position.mDeviceToAbsoluteTracking.m11, 1);
                Mat4x4 = Matrix4x4.Transform(Mat4x4, new Quaternion(.707106f, 0, 0, .707106f));
                Mat4x4 = Matrix4x4.Transform(Mat4x4, new Quaternion(0, 0, .707106f, .707106f));
                Quat1 = System.Numerics.Quaternion.CreateFromRotationMatrix(Mat4x4);
                EGMVector3 controller1Position = new EGMVector3(Mat4x4.M41, Mat4x4.M42, Mat4x4.M43);
                BoundProperties.VRControllerPosition = controller1Position;
                Vector3 rot = QuatToEuler(Quat1);
                BoundProperties.VRControllerRotation = new EGMVector3(rot.X, rot.Y, rot.Z);

                Mat4x4 = new System.Numerics.Matrix4x4(
                    position2.mDeviceToAbsoluteTracking.m0, position2.mDeviceToAbsoluteTracking.m4, position2.mDeviceToAbsoluteTracking.m8, 0,
                    position2.mDeviceToAbsoluteTracking.m1, position2.mDeviceToAbsoluteTracking.m5, position2.mDeviceToAbsoluteTracking.m9, 0,
                    position2.mDeviceToAbsoluteTracking.m2, position2.mDeviceToAbsoluteTracking.m6, position2.mDeviceToAbsoluteTracking.m10, 0,
                    position2.mDeviceToAbsoluteTracking.m3, position2.mDeviceToAbsoluteTracking.m7, position2.mDeviceToAbsoluteTracking.m11, 1);
                Mat4x4 = Matrix4x4.Transform(Mat4x4, new Quaternion(.707106f, 0, 0, .707106f));
                Mat4x4 = Matrix4x4.Transform(Mat4x4, new Quaternion(0, 0, .707106f, .707106f));
                Quat2 = System.Numerics.Quaternion.CreateFromRotationMatrix(Mat4x4);
                EGMVector3 controller2Position = new EGMVector3(Mat4x4.M41, Mat4x4.M42, Mat4x4.M43);
                BoundProperties.VRControllerPosition2 = controller2Position;
                rot = QuatToEuler(Quat2);
                BoundProperties.VRControllerRotation2 = new EGMVector3(rot.X, rot.Y, rot.Z);

                //if (startRobot)
                //{
                    //if (controllerState.rAxis1.x == 1)
                    //{
                    //    grip.Set();
                    //}
                    //else
                    //{
                    //    grip.Reset();
                    //}
                    switch (EGMCommunication.Move_Type)
                    {
                        case EGM_6_10.UDPUC_RW6_10.MotionType.Euler:
                            if (controllerState.rAxis2.x >= .9f)
                            {
                                Vector3 rotation = QuatToEuler(Quat1);
                                EGMCommunication.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Euler;
                                EGMCommunication.SetEularPose((position.mDeviceToAbsoluteTracking.m3 - StartPose.mDeviceToAbsoluteTracking.m3) * 1000,
                            (position.mDeviceToAbsoluteTracking.m7 - StartPose.mDeviceToAbsoluteTracking.m7) * 1000,
                            (position.mDeviceToAbsoluteTracking.m11 - StartPose.mDeviceToAbsoluteTracking.m11) * 1000,
                            rotation.X, rotation.Y, rotation.Z);
                            }
                            else
                            {
                                double[] currentPos = EGMCommunication.CurrentPose;
                                EGMCommunication.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
                                EGMCommunication.SetOrientPose(currentPos[0], currentPos[1], currentPos[2], currentPos[3], currentPos[4], currentPos[5], currentPos[6]);

                            }
                            break;
                        case EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion:
                            //if (controllerState.rAxis2.x >= .9f)
                            //{
                            EGMCommunication.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
                            EGMCommunication.SetOrientPose((controller1Position.X - startPos.X) * 1000,(controller1Position.Y - startPos.Y) * 1000,(controller1Position.Z - startPos.Z) * 1000,
                            Quat1.W, Quat1.X, Quat1.Y, Quat1.Z);
                        //Console.WriteLine(Quat1.ToString());
                            //}
                            //        else
                            //{
                            //    double[] currentPos = EGMCommunication.CurrentPose;
                            //    EGMCommunication.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
                            //    EGMCommunication.SetOrientPose(currentPos[0], currentPos[1], currentPos[2], currentPos[3], currentPos[4], currentPos[5], currentPos[6]);
                            //}

                            break;
                        case EGM_6_10.UDPUC_RW6_10.MotionType.Joint:
                            throw new NotImplementedException("I don't feel like having you break the robot, too bad so sad cry me a river");
                    }

                //}

            } while (true);
        }

        public Vector3 QuatToEuler(Quaternion q)
        {
            var rotation = q;
            double q0 = rotation.W;
            double q1 = rotation.Y;
            double q2 = rotation.X;
            double q3 = rotation.Z;
            Vector3 radAngles = new Vector3();
            radAngles.Y = -(float)Math.Atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (Math.Pow(q1, 2) + Math.Pow(q2, 2))) * 57.2958f;
            radAngles.X = -(float)Math.Asin(2 * (q0 * q2 - q3 * q1)) * 57.2958f;
            radAngles.Z = (float)Math.Atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (Math.Pow(q2, 2) + Math.Pow(q3, 2))) * 57.2958f;

            return radAngles;
        }
        private void Window_Closed(object sender, EventArgs e)
        {
            if (EGMCommunication != null)
            {
                EGMCommunication.Stop();
            }

            if (updateThread != null)
            {
                updateThread.Abort();
            }


            OpenVR.Shutdown();

        }

        private void CboFoundControllers_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ActiveController = Controller.Connect((ControllerInfo)((ComboBox)sender).SelectedItem, ConnectionType.Standalone);
            ActiveController.ConnectionChanged += ActiveController_ConnectionChanged;
            BoundProperties.RobotConnected = ActiveController.Connected;
            ActiveController.OperatingModeChanged += ActiveController_OperatingModeChanged;
            BoundProperties.RobotOperatingMode = ActiveController.OperatingMode;
            ActiveController.StateChanged += ActiveController_StateChanged;
            BoundProperties.RobotControllerState = ActiveController.State;

        }


        private void ActiveController_StateChanged(object sender, StateChangedEventArgs e)
        {
            BoundProperties.RobotControllerState = e.NewState;
        }

        private void ActiveController_OperatingModeChanged(object sender, OperatingModeChangeEventArgs e)
        {
            BoundProperties.RobotOperatingMode = e.NewMode;
        }

        private void ActiveController_ConnectionChanged(object sender, ConnectionChangedEventArgs e)
        {
            BoundProperties.RobotConnected = e.Connected;
        }

        private void btnRetryVRConnection_Click(object sender, RoutedEventArgs e)
        {
            //VRInitialize();
        }

        private void btnStartStreaming_Click(object sender, RoutedEventArgs e)
        {
            updateThread = new Thread(VRThread);
            updateThread.Start();
        }
    }

}
