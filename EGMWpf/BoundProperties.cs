using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.ComponentModel;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers;
using System.Windows.Media;
using Valve.VR;
using System.Numerics;
namespace EGMWpf
{
	class BoundProperties : INotifyPropertyChanged
	{
		public event PropertyChangedEventHandler PropertyChanged;
		protected void OnPropertyChanged(string propertyName)
		{
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
		}

		private ControllerInfoCollection _FoundControllers;
		public ControllerInfoCollection FoundControllers
		{
			get
			{
				return _FoundControllers;
			}
			set 
			{
				_FoundControllers = value;
				OnPropertyChanged(nameof(FoundControllers));
			}
		}
		private bool _RobotConnected;
		public bool RobotConnected
        {
            get
            {
				return _RobotConnected;
            }
            set
            {
				_RobotConnected = value;
				OnPropertyChanged(nameof(RobotConnected));
				OnPropertyChanged(nameof(RobotConnectionBrush));
            }
        }
		public Brush RobotConnectionBrush
        {
            get
            {
				if (RobotConnected == true)
				{
					return Brushes.LightGreen;
				}
                else
                {
					return Brushes.Red;
                }
            }
        }
		private ControllerOperatingMode _RobotOperatingMode;
		public ControllerOperatingMode RobotOperatingMode
        {
            get
            {
				return _RobotOperatingMode;
            }
            set
            {
				_RobotOperatingMode = value;
				OnPropertyChanged(nameof(RobotOperatingMode));
			}
        }
		private ControllerState _RobotControllerState;
		public ControllerState RobotControllerState
        {
            get
            {
				return _RobotControllerState;
            }
            set
            {
				_RobotControllerState = value;
				OnPropertyChanged(nameof(RobotControllerState));
			}
        }
		private EVRInitError _VRInitializeError;
		public EVRInitError VRInitializeError
        {
            get
            {
				return _VRInitializeError;
            }
            set
            {
				_VRInitializeError = value;
				OnPropertyChanged(nameof(VRInitializeError));
				OnPropertyChanged(nameof(VRConnectionBrush));
			}
        }
		public Brush VRConnectionBrush
		{
			get
			{
				if (VRInitializeError ==  EVRInitError.None )
				{
					return Brushes.LightGreen;
				}
				else
				{
					
					return Brushes.Red;
				}
			}
		}
		
		private EGMVector3 _VRControllerPosition;
		public EGMVector3 VRControllerPosition
        {
            get
            {
				return _VRControllerPosition;
            }
            set
            {
				
				_VRControllerPosition = value;
				OnPropertyChanged(nameof(VRControllerPosition));
			}
        }
		private EGMVector3 _VRControllerRotation;
		public EGMVector3 VRControllerRotation
		{
			get
			{
				return _VRControllerRotation;
			}
			set
			{
				_VRControllerRotation = value;
				OnPropertyChanged(nameof(VRControllerRotation));
			}
		}
		private EGMVector3 _VRControllerPosition2;
		public EGMVector3 VRControllerPosition2
		{
			get
			{
				return _VRControllerPosition2;
			}
			set
			{

				_VRControllerPosition2 = value;
				OnPropertyChanged(nameof(VRControllerPosition2));
			}
		}
		private EGMVector3 _VRControllerRotation2;
		public EGMVector3 VRControllerRotation2
		{
			get
			{
				return _VRControllerRotation2;
			}
			set
			{
				_VRControllerRotation2 = value;
				OnPropertyChanged(nameof(VRControllerRotation2));
			}
		}
		private EGMQuaternionPose _RobotOneQuaternionPose;
		public EGMQuaternionPose RobotOneQuaternionPose
        {
            get
            {
				return _RobotOneQuaternionPose;
            }
            set
            {
				_RobotOneQuaternionPose = value;
				OnPropertyChanged(nameof(RobotOneQuaternionPose));
			}
        }
		private EGMQuaternionPose _RobotTwoQuaternionPose;
		public EGMQuaternionPose RobotTwoQuaternionPose
		{
			get
			{
				return _RobotTwoQuaternionPose;
			}
			set
			{
				_RobotTwoQuaternionPose = value;
				OnPropertyChanged(nameof(RobotTwoQuaternionPose));
			}
		}
	}
}
