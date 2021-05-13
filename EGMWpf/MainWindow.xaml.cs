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
        EGM_6_10.UDPUC_RW6_10 EGMPingCom;
        EGM_6_10.UDPUC_RW6_10 EGMPongCom;
        ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal PingGrip;
        ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal PongGrip;
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
            EGMPingCom = new EGM_6_10.UDPUC_RW6_10(6510, EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion);
            EGMPingCom.Start();
            EGMPongCom = new EGM_6_10.UDPUC_RW6_10(6511, EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion);
            EGMPongCom.Start();
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
            uint ControllerRIndex = 1;
            uint ControllerLIndex = 2;
            Quaternion PingQuat;
            Quaternion PongQuat;
            Matrix4x4 Mat4x4;
            VRControllerState_t BeginState = new VRControllerState_t();
            VRControllerState_t BeginState2 = new VRControllerState_t();
            TrackedDevicePose_t StartPingPose = new TrackedDevicePose_t();
            TrackedDevicePose_t StartPongPose = new TrackedDevicePose_t();
            headset.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerLIndex, ref BeginState, BeginState.unPacketNum, ref StartPingPose);
            headset.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerRIndex, ref BeginState2, BeginState.unPacketNum, ref StartPongPose);
            Vector3 StartPingVec = new Vector3(StartPingPose.mDeviceToAbsoluteTracking.m3, StartPingPose.mDeviceToAbsoluteTracking.m7, StartPingPose.mDeviceToAbsoluteTracking.m11);
            StartPingVec = Vector3.Transform(StartPingVec, new Quaternion(.707106f, 0, 0, .707106f));
            StartPingVec = Vector3.Transform(StartPingVec, new Quaternion(0, 0, .707106f, .707106f));

            Vector3 StartPongVec = new Vector3(StartPongPose.mDeviceToAbsoluteTracking.m3, StartPongPose.mDeviceToAbsoluteTracking.m7, StartPongPose.mDeviceToAbsoluteTracking.m11);
            StartPongVec = Vector3.Transform(StartPongVec, new Quaternion(.707106f, 0, 0, .707106f));
            StartPongVec = Vector3.Transform(StartPongVec, new Quaternion(0, 0, .707106f, .707106f));

            do
            {

                //TrackedDevicePose_t[] t = new TrackedDevicePose_t[6];
                //headset.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseStanding, 0, t);
                VRControllerState_t LeftControllerState = new VRControllerState_t();
                VRControllerState_t RightControllerState = new VRControllerState_t();
                TrackedDevicePose_t LContPose = new TrackedDevicePose_t();
                TrackedDevicePose_t RContPose = new TrackedDevicePose_t();

                headset.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerLIndex, ref LeftControllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)), ref LContPose);
                headset.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerRIndex, ref RightControllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)), ref RContPose);

                Mat4x4 = new System.Numerics.Matrix4x4(
                    LContPose.mDeviceToAbsoluteTracking.m0, LContPose.mDeviceToAbsoluteTracking.m4, LContPose.mDeviceToAbsoluteTracking.m8, 0,
                    LContPose.mDeviceToAbsoluteTracking.m1, LContPose.mDeviceToAbsoluteTracking.m5, LContPose.mDeviceToAbsoluteTracking.m9, 0,
                    LContPose.mDeviceToAbsoluteTracking.m2, LContPose.mDeviceToAbsoluteTracking.m6, LContPose.mDeviceToAbsoluteTracking.m10, 0,
                    LContPose.mDeviceToAbsoluteTracking.m3, LContPose.mDeviceToAbsoluteTracking.m7, LContPose.mDeviceToAbsoluteTracking.m11, 1);
                Mat4x4 = Matrix4x4.Transform(Mat4x4, new Quaternion(.707106f, 0, 0, .707106f));
                Mat4x4 = Matrix4x4.Transform(Mat4x4, new Quaternion(0, 0, .707106f, .707106f));
                PingQuat = System.Numerics.Quaternion.CreateFromRotationMatrix(Mat4x4);
                EGMVector3 PingContPos = new EGMVector3(Mat4x4.M41, Mat4x4.M42, Mat4x4.M43);
                BoundProperties.VRControllerPosition = PingContPos;
                Vector3 rot = QuatToEuler(PingQuat);
                BoundProperties.VRControllerRotation = new EGMVector3(rot.X, rot.Y, rot.Z);

                Mat4x4 = new System.Numerics.Matrix4x4(
                    RContPose.mDeviceToAbsoluteTracking.m0, RContPose.mDeviceToAbsoluteTracking.m4, RContPose.mDeviceToAbsoluteTracking.m8, 0,
                    RContPose.mDeviceToAbsoluteTracking.m1, RContPose.mDeviceToAbsoluteTracking.m5, RContPose.mDeviceToAbsoluteTracking.m9, 0,
                    RContPose.mDeviceToAbsoluteTracking.m2, RContPose.mDeviceToAbsoluteTracking.m6, RContPose.mDeviceToAbsoluteTracking.m10, 0,
                    RContPose.mDeviceToAbsoluteTracking.m3, RContPose.mDeviceToAbsoluteTracking.m7, RContPose.mDeviceToAbsoluteTracking.m11, 1);
                Mat4x4 = Matrix4x4.Transform(Mat4x4, new Quaternion(.707106f, 0, 0, .707106f));
                Mat4x4 = Matrix4x4.Transform(Mat4x4, new Quaternion(0, 0, .707106f, .707106f));
                PongQuat = System.Numerics.Quaternion.CreateFromRotationMatrix(Mat4x4);
                EGMVector3 PongContPos = new EGMVector3(Mat4x4.M41, Mat4x4.M42, Mat4x4.M43);
                BoundProperties.VRControllerPosition2 = PongContPos;
                rot = QuatToEuler(PongQuat);
                BoundProperties.VRControllerRotation2 = new EGMVector3(rot.X, rot.Y, rot.Z);


                if (LeftControllerState.rAxis1.x >.5)
                {
                    PingGrip.Set();
                }
                else
                {
                    PingGrip.Reset();
                }

                if (RightControllerState.rAxis1.x >.5 )
                {
                    PongGrip.Set();
                }
                else
                {
                    PongGrip.Reset();
                }


                switch (EGMPingCom.Move_Type)
                {
                    case EGM_6_10.UDPUC_RW6_10.MotionType.Euler:
                        if (LeftControllerState.rAxis2.x >= .9f)
                        {
                            Vector3 rotation = QuatToEuler(PingQuat);
                            EGMPingCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Euler;
                            EGMPingCom.SetEularPose((LContPose.mDeviceToAbsoluteTracking.m3 - StartPingPose.mDeviceToAbsoluteTracking.m3) * 1000,
                        (LContPose.mDeviceToAbsoluteTracking.m7 - StartPingPose.mDeviceToAbsoluteTracking.m7) * 1000,
                        (LContPose.mDeviceToAbsoluteTracking.m11 - StartPingPose.mDeviceToAbsoluteTracking.m11) * 1000,
                        rotation.X, rotation.Y, rotation.Z);
                        }
                        else
                        {
                            double[] currentPos = EGMPingCom.CurrentPose;
                            EGMPingCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
                            EGMPingCom.SetOrientPose(currentPos[0], currentPos[1], currentPos[2], currentPos[3], currentPos[4], currentPos[5], currentPos[6]);

                        }
                        break;
                    case EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion:
                        //if (controllerState.rAxis2.x >= .9f)
                        //{
                        EGMPingCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
                        EGMPingCom.SetOrientPose((PingContPos.X - StartPingVec.X) * 1000, (PingContPos.Y - StartPingVec.Y) * 1000, (PingContPos.Z - StartPingVec.Z) * 1000,
                        PingQuat.W, PingQuat.X, PingQuat.Y, PingQuat.Z);
                        //Console.WriteLine(Quat1.ToString());

                        break;
                    case EGM_6_10.UDPUC_RW6_10.MotionType.Joint:
                        throw new NotImplementedException("I don't feel like having you break the robot, too bad so sad cry me a river");
                }



                switch (EGMPongCom.Move_Type)
                {
                    case EGM_6_10.UDPUC_RW6_10.MotionType.Euler:
                        if (LeftControllerState.rAxis2.x >= .9f)
                        {
                            Vector3 rotation = QuatToEuler(PongQuat);
                            EGMPongCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Euler;
                            EGMPongCom.SetEularPose((LContPose.mDeviceToAbsoluteTracking.m3 - StartPongPose.mDeviceToAbsoluteTracking.m3) * 1000,
                        (LContPose.mDeviceToAbsoluteTracking.m7 - StartPongPose.mDeviceToAbsoluteTracking.m7) * 1000,
                        (LContPose.mDeviceToAbsoluteTracking.m11 - StartPongPose.mDeviceToAbsoluteTracking.m11) * 1000,
                        rotation.X, rotation.Y, rotation.Z);
                        }
                        else
                        {
                            double[] currentPos = EGMPongCom.CurrentPose;
                            EGMPongCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
                            EGMPongCom.SetOrientPose(currentPos[0], currentPos[1], currentPos[2], currentPos[3], currentPos[4], currentPos[5], currentPos[6]);

                        }
                        break;
                    case EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion:
                        //if (controllerState.rAxis2.x >= .9f)
                        //{
                        EGMPongCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
                        EGMPongCom.SetOrientPose((PongContPos.X - StartPongVec.X) * 1000, (PongContPos.Y - StartPongVec.Y) * 1000, (PongContPos.Z - StartPongVec.Z) * 1000,
                        PongQuat.W, PongQuat.X, PongQuat.Y, PongQuat.Z);
                        //Console.WriteLine(Quat1.ToString());

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
            if (EGMPingCom != null)
            {
                EGMPingCom.Stop();
            }

            if (EGMPongCom != null)
            {
                EGMPongCom.Stop();
            }

            if (updateThread != null)
            {
                updateThread.Abort();
            }


            OpenVR.Shutdown();
            Thread.Sleep(1000);
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

            try
            {
                PingGrip = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal) ActiveController.IOSystem.GetSignal("PING_VALVE_1");
                PongGrip = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal)ActiveController.IOSystem.GetSignal("PONG_VALVE_1");
            }
            catch (Exception errmsg)
            {
                Console.WriteLine(errmsg.Message);
            }

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
