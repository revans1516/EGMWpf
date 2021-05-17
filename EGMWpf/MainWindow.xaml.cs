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
using ABB.Robotics.Controllers.RapidDomain;

namespace EGMWpf
{
	/// <summary>
	/// Interaction logic for MainWindow.xaml
	/// </summary>
	public partial class MainWindow : Window
	{
		
		Thread VRStreamingThread;
		Thread IOControlThread;
		Thread RobotPositionThread;
		CVRSystem OpenVRConnection;
		//Initializing variables for Ping and Pong
		uint ControllerRIndex = 2;
		uint ControllerLIndex = 1;
		EGM_6_10.UDPUC_RW6_10 EGMPingCom;
		EGM_6_10.UDPUC_RW6_10 EGMPongCom;
		ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal PingGripOpen;
		ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal PongGripOpen;

		ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal PingGripClose;
		ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal PongGripClose;
		Controller ActiveController;
		BoundProperties BoundProperties = new BoundProperties();
		bool updateRobotPosition = false;
		bool streamVRData = false;
		public MainWindow()
		{
			InitializeComponent();
			//StartPolhemus();
			//return;
			//Finds all controllers on network, specifically looks for Virtual Controllers on computer as well
			NetworkScanner scanner = new NetworkScanner();
			NetworkScanner.AddRemoteController("127.0.0.1");
			scanner.Scan();
			VRStreamingThread = new Thread(VRThread);
			ControllerInfoCollection controllers = scanner.Controllers;
			DataContext = BoundProperties;
			BoundProperties.FoundControllers = controllers;
			//Initializes the VR connection
			VRInitialize();
			//Connects to the EGM ports of Ping and Pong
			EGMPingCom = new EGM_6_10.UDPUC_RW6_10(6510, EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion);
			EGMPingCom.Start();
			EGMPongCom = new EGM_6_10.UDPUC_RW6_10(6511, EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion);
			EGMPongCom.Start();
		}
		private void VRInitialize()
		{
			EVRInitError error = new EVRInitError();
			//Initializes OpenVR. Returns the connection, if the connection fails, returns why the connection failed
			OpenVRConnection = OpenVR.Init(ref error, EVRApplicationType.VRApplication_Background);
			BoundProperties.VRInitializeError = error;
			//If an error has occured, allows the user to retry connection
			if (error != EVRInitError.None)
			{
				btnRetryConnection.Visibility = Visibility.Visible;
				btnStartStreaming.Visibility = Visibility.Hidden;
			}
			else
			{
				btnRetryConnection.Visibility = Visibility.Hidden;
				btnStartStreaming.Visibility = Visibility.Visible;
			}
		}
		public void VRThread()
		{

			Quaternion PingQuat=new Quaternion();
			Quaternion PongQuat= new Quaternion();
			Matrix4x4 PingMatrix;
			Matrix4x4 PongMatrix;
			VRControllerState_t LeftControllerState = new VRControllerState_t();
			VRControllerState_t RightControllerState = new VRControllerState_t();
			TrackedDevicePose_t LContPose = new TrackedDevicePose_t();
			TrackedDevicePose_t RContPose = new TrackedDevicePose_t();
			//VRControllerState_t BeginState = new VRControllerState_t();
			//VRControllerState_t BeginState2 = new VRControllerState_t();
			//TrackedDevicePose_t StartPingPose = new TrackedDevicePose_t();
			//TrackedDevicePose_t StartPongPose = new TrackedDevicePose_t();
			//Gets the initial state for the VR controllers. This is used to zero out the coordinate system
			//OpenVRConnection.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerLIndex, ref BeginState, BeginState.unPacketNum, ref StartPingPose);
			//OpenVRConnection.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerRIndex, ref BeginState2, BeginState.unPacketNum, ref StartPongPose);
			OpenVRConnection.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerLIndex, ref LeftControllerState, LeftControllerState.unPacketNum, ref LContPose);
			OpenVRConnection.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerRIndex, ref RightControllerState, RightControllerState.unPacketNum, ref RContPose);
			//ulong VideoHandle=0;
			//var FrontCamera = OpenVR.TrackedCamera;
			//FrontCamera.AcquireVideoStreamingService(0, ref VideoHandle);
			//EVRSettingsError e = EVRSettingsError.None;
			//OpenVR.Settings.SetBool(OpenVR.k_pch_Camera_Section, OpenVR.k_pch_Camera_EnableCameraForCollisionBounds_Bool, true,ref e );

			Vector3 StartPingVec = new Vector3(LContPose.mDeviceToAbsoluteTracking.m3, LContPose.mDeviceToAbsoluteTracking.m7, LContPose.mDeviceToAbsoluteTracking.m11);
			//Rotates the inital position to match the coordinate system of the robots
			StartPingVec = Vector3.Transform(StartPingVec, new Quaternion(.707106f, 0, 0, .707106f));
			StartPingVec = Vector3.Transform(StartPingVec, new Quaternion(0, 0, .707106f, .707106f));

			Vector3 StartPongVec = new Vector3(RContPose.mDeviceToAbsoluteTracking.m3, RContPose.mDeviceToAbsoluteTracking.m7, RContPose.mDeviceToAbsoluteTracking.m11);
			StartPongVec = Vector3.Transform(StartPongVec, new Quaternion(.707106f, 0, 0, .707106f));
			StartPongVec = Vector3.Transform(StartPongVec, new Quaternion(0, 0, .707106f, .707106f));
			EGMVector3 PingRot = new EGMVector3(QuatToEuler(PingQuat));
			EGMVector3 PingContPos = new EGMVector3(0, 0, 0);

			EGMVector3 PongRot = new EGMVector3(QuatToEuler(PongQuat));
			EGMVector3 PongContPos = new EGMVector3(0,0,0);

			BoundProperties.VRControllerPosition = PingContPos;
			BoundProperties.VRControllerRotation = PingRot;

			BoundProperties.VRControllerPosition2 = PongContPos;
			BoundProperties.VRControllerRotation2 = PongRot;

			do
			{

				
				//Gets the current states of the controllers. This includes their positions
				OpenVRConnection.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerLIndex, ref LeftControllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)), ref LContPose);
				OpenVRConnection.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerRIndex, ref RightControllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)), ref RContPose);
				//Constructs a matrix from the controller state
				PingMatrix = new System.Numerics.Matrix4x4(
					LContPose.mDeviceToAbsoluteTracking.m0, LContPose.mDeviceToAbsoluteTracking.m4, LContPose.mDeviceToAbsoluteTracking.m8, 0,
					LContPose.mDeviceToAbsoluteTracking.m1, LContPose.mDeviceToAbsoluteTracking.m5, LContPose.mDeviceToAbsoluteTracking.m9, 0,
					LContPose.mDeviceToAbsoluteTracking.m2, LContPose.mDeviceToAbsoluteTracking.m6, LContPose.mDeviceToAbsoluteTracking.m10, 0,
					LContPose.mDeviceToAbsoluteTracking.m3, LContPose.mDeviceToAbsoluteTracking.m7, LContPose.mDeviceToAbsoluteTracking.m11, 1);
				//Rotates the matrix to fit the coordinate system of the robot
				PingMatrix = Matrix4x4.Transform(PingMatrix, new Quaternion(.707106f, 0, 0, .707106f));
				PingMatrix = Matrix4x4.Transform(PingMatrix, new Quaternion(0, 0, .707106f, .707106f));
				PingQuat = Quaternion.CreateFromRotationMatrix(PingMatrix);
				PingContPos = new EGMVector3(PingMatrix.M41, PingMatrix.M42, PingMatrix.M43);
				PingRot = new EGMVector3(QuatToEuler(PingQuat));
				//BoundProperties.VRControllerPosition = PingContPos;
				//BoundProperties.VRControllerRotation = PingRot;



				//Constructs a matrix from the controller state
				PongMatrix = new System.Numerics.Matrix4x4(
					RContPose.mDeviceToAbsoluteTracking.m0, RContPose.mDeviceToAbsoluteTracking.m4, RContPose.mDeviceToAbsoluteTracking.m8, 0,
					RContPose.mDeviceToAbsoluteTracking.m1, RContPose.mDeviceToAbsoluteTracking.m5, RContPose.mDeviceToAbsoluteTracking.m9, 0,
					RContPose.mDeviceToAbsoluteTracking.m2, RContPose.mDeviceToAbsoluteTracking.m6, RContPose.mDeviceToAbsoluteTracking.m10, 0,
					RContPose.mDeviceToAbsoluteTracking.m3, RContPose.mDeviceToAbsoluteTracking.m7, RContPose.mDeviceToAbsoluteTracking.m11, 1);
				//Rotates the matrix to fit the coordinate system of the robot
				PongMatrix = Matrix4x4.Transform(PongMatrix, new Quaternion(.707106f, 0, 0, .707106f));
				PongMatrix = Matrix4x4.Transform(PongMatrix, new Quaternion(0, 0, .707106f, .707106f));
				PongQuat = Quaternion.CreateFromRotationMatrix(PongMatrix);
				PongContPos = new EGMVector3(PongMatrix.M41, PongMatrix.M42, PongMatrix.M43);
				PongRot = new EGMVector3(QuatToEuler(PongQuat));
				//BoundProperties.VRControllerPosition2 = PongContPos;
				//BoundProperties.VRControllerRotation2 = PongRot;






				double[] PingControllerPosition = EGMPingCom.CurrentPose;
				double[] PongControllerPosition = EGMPongCom.CurrentPose;
				BoundProperties.RobotOneQuaternionPose = new EGMQuaternionPose(PingControllerPosition[0], PingControllerPosition[1], PingControllerPosition[2], PingControllerPosition[3], PingControllerPosition[4], PingControllerPosition[5], PingControllerPosition[6]);
				BoundProperties.RobotTwoQuaternionPose = new EGMQuaternionPose(PongControllerPosition[0], PongControllerPosition[1], PongControllerPosition[2], PongControllerPosition[3], PongControllerPosition[4], PongControllerPosition[5], PongControllerPosition[6]);
				switch (EGMPingCom.Move_Type)
				{
					case EGM_6_10.UDPUC_RW6_10.MotionType.Euler:

						if (LeftControllerState.rAxis2.x >= .9f)
						{
							EGMPingCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Euler;
							EGMPingCom.SetEularPose((PingContPos.X - StartPingVec.X) * 1000,
						(PingContPos.Y - StartPingVec.Y) * 1000,
						(PingContPos.Z - StartPingVec.Z) * 1000,
						PingRot.X, PingRot.Y, PingRot.Z);

						}
						else
						{
							StartPingVec = new Vector3(LContPose.mDeviceToAbsoluteTracking.m3, LContPose.mDeviceToAbsoluteTracking.m7, LContPose.mDeviceToAbsoluteTracking.m11);
							EGMPingCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
							EGMPingCom.SetOrientPose(PingControllerPosition[0], PingControllerPosition[1], PingControllerPosition[2], PingControllerPosition[3], PingControllerPosition[4], PingControllerPosition[5], PingControllerPosition[6]);

						}
						break;
					case EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion:
						if (LeftControllerState.rAxis2.x >= .9f)
						{
							EGMPingCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
							EGMPingCom.SetOrientPose((PingContPos.X - StartPingVec.X) * 1000, (PingContPos.Y - StartPingVec.Y) * 1000, (PingContPos.Z - StartPingVec.Z) * 1000,
							PingQuat.W, PingQuat.X, PingQuat.Y, PingQuat.Z);
						}
						else
						{
							StartPingVec = new Vector3(LContPose.mDeviceToAbsoluteTracking.m3, LContPose.mDeviceToAbsoluteTracking.m7, LContPose.mDeviceToAbsoluteTracking.m11);
							EGMPingCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
							EGMPingCom.SetOrientPose(PingControllerPosition[0], PingControllerPosition[1], PingControllerPosition[2], PingControllerPosition[3], PingControllerPosition[4], PingControllerPosition[5], PingControllerPosition[6]);
						}
						break;
					case EGM_6_10.UDPUC_RW6_10.MotionType.Joint:
						throw new NotImplementedException("I don't feel like having you break the robot, too bad so sad cry me a river");
				}



				switch (EGMPongCom.Move_Type)
				{
					case EGM_6_10.UDPUC_RW6_10.MotionType.Euler:

						if (RightControllerState.rAxis2.x >= .9f)
						{

							Vector3 rotation = QuatToEuler(PongQuat);
							EGMPongCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Euler;
							EGMPongCom.SetEularPose((PongContPos.X - StartPongVec.X) * 1000,
						(PongContPos.Y - StartPongVec.Y) * 1000,
						(PongContPos.Z - StartPongVec.Z) * 1000,
						PongRot.X, PongRot.Y, PongRot.Z);

						}
						else
						{
							StartPongVec = new Vector3(RContPose.mDeviceToAbsoluteTracking.m3, RContPose.mDeviceToAbsoluteTracking.m7, RContPose.mDeviceToAbsoluteTracking.m11);
							EGMPongCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
							EGMPongCom.SetOrientPose(PongControllerPosition[0], PongControllerPosition[1], PongControllerPosition[2], PongControllerPosition[3], PongControllerPosition[4], PongControllerPosition[5], PongControllerPosition[6]);


						}
						break;
					case EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion:
						if (RightControllerState.rAxis2.x >= .9f)
						{
							EGMPongCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
							EGMPongCom.SetOrientPose((PongContPos.X - StartPongVec.X) * 1000, (PongContPos.Y - StartPongVec.Y) * 1000, (PongContPos.Z - StartPongVec.Z) * 1000,
							PongQuat.W, PongQuat.X, PongQuat.Y, PongQuat.Z);
						}
						else
						{
							StartPongVec = new Vector3(RContPose.mDeviceToAbsoluteTracking.m3, RContPose.mDeviceToAbsoluteTracking.m7, RContPose.mDeviceToAbsoluteTracking.m11);
							EGMPongCom.Move_Type = EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion;
							EGMPongCom.SetOrientPose(PongControllerPosition[0], PongControllerPosition[1], PongControllerPosition[2], PongControllerPosition[3], PongControllerPosition[4], PongControllerPosition[5], PongControllerPosition[6]);
						}

						break;
					case EGM_6_10.UDPUC_RW6_10.MotionType.Joint:
						throw new NotImplementedException("I don't feel like having you break the robot, too bad so sad cry me a river");
				}


			} while (streamVRData);
		}

		public Vector3 QuatToEuler(Quaternion q)
		{
			var rotation = q;
			double q0 = rotation.W;
			double q1 = rotation.Y;
			double q2 = rotation.X;
			double q3 = rotation.Z;
			Vector3 radAngles = new Vector3();
			//Angles from the controllers are in YXZ Format. 
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

			if (VRStreamingThread != null && VRStreamingThread.IsAlive)
			{
				streamVRData = false;
				VRStreamingThread.Join();
				VRStreamingThread.Abort();
			}

			//if (RobotPositionThread != null)
			//{
			//    updateRobotPosition = false;
			//    RobotPositionThread.Abort();
			//}
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
			RobTarget RobotOnePosition = ActiveController.MotionSystem.MechanicalUnits[0].GetPosition(ABB.Robotics.Controllers.MotionDomain.CoordinateSystemType.World);
			BoundProperties.RobotOneQuaternionPose = new EGMQuaternionPose(RobotOnePosition.Trans.X, RobotOnePosition.Trans.Y, RobotOnePosition.Trans.Z, RobotOnePosition.Rot.Q1, RobotOnePosition.Rot.Q2, RobotOnePosition.Rot.Q3, RobotOnePosition.Rot.Q4);
			RobTarget RobotTwoPosition = ActiveController.MotionSystem.MechanicalUnits[1].GetPosition(ABB.Robotics.Controllers.MotionDomain.CoordinateSystemType.World);
			BoundProperties.RobotTwoQuaternionPose = new EGMQuaternionPose(RobotTwoPosition.Trans.X, RobotTwoPosition.Trans.Y, RobotTwoPosition.Trans.Z, RobotTwoPosition.Rot.Q1, RobotTwoPosition.Rot.Q2, RobotTwoPosition.Rot.Q3, RobotTwoPosition.Rot.Q4);

			//updateRobotPosition = true;
			//RobotPositionThread = new Thread(RobotPositionUpdater);
			//RobotPositionThread.Start();
			try
			{
				PingGripClose = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal)ActiveController.IOSystem.GetSignal("PING_VALVE_1");
				PongGripClose = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal)ActiveController.IOSystem.GetSignal("PONG_VALVE_1");

				PingGripOpen = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal)ActiveController.IOSystem.GetSignal("PING_VALVE_2");
				PongGripOpen = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal)ActiveController.IOSystem.GetSignal("PONG_VALVE_2");
			}
			catch (Exception errmsg)
			{
				Console.WriteLine(errmsg.Message);
			}
			IOControlThread = new Thread(IOControl);
			IOControlThread.Start();
		}
		
		private void IOControl()
		{
			do
			{
				VRControllerState_t LeftControllerState = new VRControllerState_t();
				VRControllerState_t RightControllerState = new VRControllerState_t();
				OpenVRConnection.GetControllerState(ControllerLIndex, ref LeftControllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)));
				OpenVRConnection.GetControllerState(ControllerRIndex, ref RightControllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)));
				//OpenVRConnection.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerLIndex, ref LeftControllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)), ref LContPose);
				//OpenVRConnection.GetControllerStateWithPose(ETrackingUniverseOrigin.TrackingUniverseStanding, ControllerRIndex, ref RightControllerState, (uint)Marshal.SizeOf(typeof(VRControllerState_t)), ref RContPose);
				//Triggering the grippers on the robots if the trigger on the controller is held
				if (LeftControllerState.rAxis1.x < .5)
				{
					PingGripOpen.Reset();
					PingGripClose.Set();

				}
				else
				{
					PingGripClose.Reset();
					PingGripOpen.Set();
				}

				if (RightControllerState.rAxis1.x < .5)
				{
					PongGripOpen.Reset();
					PongGripClose.Set();
				}
				else
				{
					PongGripClose.Reset();
					PongGripOpen.Set();

				}
			} while (true);

		}
		private void StartPolhemus()
		{
			PlStream plStream = new PlStream(PlTracker.Liberty);
			plStream.StartReading();
			
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
			VRInitialize();
		}

		private void btnStartStreaming_Click(object sender, RoutedEventArgs e)
		{
			if (VRStreamingThread != null)
			{
				if (VRStreamingThread.IsAlive == true)
				{
					streamVRData = false;
					VRStreamingThread.Join();
					VRStreamingThread.Abort();
					btnStartStreaming.Content = "Start Streaming";
				}
				else
				{
					streamVRData = true;
					VRStreamingThread = new Thread(VRThread);
					VRStreamingThread.Start();
					btnStartStreaming.Content = "Stop Streaming";
				}
			}

		}

		private void btnConnectPolhemus_Click(object sender, RoutedEventArgs e)
		{

		}

		private void btnConnectPolhemus_Unchecked(object sender, RoutedEventArgs e)
		{

		}

		private void btnStartStreaming_Unchecked(object sender, RoutedEventArgs e)
		{

		}
	}

}
