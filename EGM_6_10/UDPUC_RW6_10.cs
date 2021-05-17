using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using abb.egm;

namespace EGM_6_10
{
    public class UDPUC_RW6_10
    {

        public enum MotionType
        {
            Euler,
            Quaternion,
            Joint,
        };


        public enum GuidanceType
        {
            Guidance=3,
            Correction=4
        }

        public MotionType Move_Type = MotionType.Joint;
        private Thread _SensorThread = null;
        private UdpClient _UDPServer = null;
        private int UDPPort;
        private bool _RunThread = true;
        private uint _SeqNumber = 0;
        private double[] PoseEular = { 0, 0, 0, 0, 0, 0 };
        private double[] Joints = { 0, 0, 0, 0, 0, 0 };
        private double[] PoseQuat = { 0, 0, 0, 1, 0, 0, 0 };


        public double[] CurrentPose = { 0, 0, 0, 1, 0, 0, 0 };
        

        public UInt64 RobotTime
        {
            get { return RobotTime; }

            private set { }
        }


        public UDPUC_RW6_10(int UDPPortNum, MotionType Move)
        {
            UDPPort = UDPPortNum;
            Move_Type = Move;
        }

        public void SetEularPose(double X, double Y, double Z, double RX, double RY, double RZ)
        {
            PoseEular = new double[] { X, Y, Z, RX, RY, RZ };
        }

        public void SetOrientPose(double X, double Y, double Z, double RW, double RX, double RY, double RZ)
        {
            PoseQuat = new double[] { X, Y, Z, RW, RX, RY, RZ };
        }

        public void SetJoint(double J1, double J2, double J3, double J4, double J5, double J6)
        {
            Joints = new double[] { J1, J2, J3, J4, J5, J6 };
        }



        private void UDPUCPoseThread()
        {

            _UDPServer = new UdpClient(UDPPort);

            var remoteEP = new IPEndPoint(IPAddress.Any, UDPPort);

            while (_RunThread)
            {
                var data = _UDPServer.Receive(ref remoteEP);
                if (data != null)
                {
                    GetCurrentPos(data);
                    EgmSensor.Builder SensorData = EgmSensor.CreateBuilder();

                    switch (Move_Type)
                    {
                        case MotionType.Euler:
                            CreateEularMessage(SensorData);
                            break;
                        case MotionType.Quaternion:
                            CreateQuatMessage(SensorData);
                            break;
                        case MotionType.Joint:
                            CreateJointMessage(SensorData);
                            break;
                    }

                    using (MemoryStream MemStream = new MemoryStream())
                    {
                        EgmSensor SensorMessage = SensorData.Build();
                        SensorMessage.WriteTo(MemStream);

                        int bytesSent = _UDPServer.Send(MemStream.ToArray(), (int)MemStream.Length, remoteEP);
                    }

                }
            }
        }

        private void CreateEularMessage(EgmSensor.Builder sensor, double OptLinSpeed = -1, double OptOrientSpeed = -1)
        {
            // create a header
            EgmHeader.Builder hdr = new EgmHeader.Builder();
            EgmSpeedRef.Builder speedbuilder = new EgmSpeedRef.Builder();
            EgmCartesianSpeed.Builder CartSpeed = new EgmCartesianSpeed.Builder();

            CartSpeed.SetValue(0, OptLinSpeed);
            CartSpeed.SetValue(1, OptLinSpeed);
            CartSpeed.SetValue(2, OptLinSpeed);
            CartSpeed.SetValue(3, OptOrientSpeed);
            CartSpeed.SetValue(4, OptOrientSpeed);
            CartSpeed.SetValue(5, OptOrientSpeed);

            hdr.SetSeqno(_SeqNumber++)
                .SetTm((uint)DateTime.Now.Ticks)
                .SetMtype(EgmHeader.Types.MessageType.MSGTYPE_CORRECTION);

            sensor.SetHeader(hdr);

            // create some sensor data
            EgmPlanned.Builder planned = new EgmPlanned.Builder();
            EgmPose.Builder pos = new EgmPose.Builder();
            EgmEuler.Builder pe = new EgmEuler.Builder();
            EgmCartesian.Builder pc = new EgmCartesian.Builder();

            pc.SetX(PoseEular[0])
                .SetY(PoseEular[1])
                .SetZ(PoseEular[2]);

            pe.SetX(PoseEular[3])
               .SetY(PoseEular[4])
               .SetZ(PoseEular[5]);

            pos.SetPos(pc)
                .SetEuler(pe);

            planned.SetCartesian(pos);  // bind pos object to planned
            sensor.SetPlanned(planned); // bind planned to sensor object

            if (OptLinSpeed > 0 && OptOrientSpeed > 0) sensor.SetSpeedRef(speedbuilder);



            return;
        }



        private void CreateQuatMessage(EgmSensor.Builder sensor, double OptLinSpeed = -1, double OptOrientSpeed = -1)
        {
            // create a header
            EgmHeader.Builder hdr = new EgmHeader.Builder();
            EgmSpeedRef.Builder speedbuilder = new EgmSpeedRef.Builder();
            //EgmCartesianSpeed.Builder CartSpeed = new EgmCartesianSpeed.Builder();

            //CartSpeed.SetValue(0, OptLinSpeed);
            //CartSpeed.SetValue(1, OptLinSpeed);
            //CartSpeed.SetValue(2, OptLinSpeed);
            //CartSpeed.SetValue(3, OptOrientSpeed);
            //CartSpeed.SetValue(4, OptOrientSpeed);
            //CartSpeed.SetValue(5, OptOrientSpeed);

            hdr.SetSeqno(_SeqNumber++)
                .SetTm((uint)DateTime.Now.Ticks)
                .SetMtype(EgmHeader.Types.MessageType.MSGTYPE_CORRECTION);

            sensor.SetHeader(hdr);

            // create some sensor data
            EgmPlanned.Builder planned = new EgmPlanned.Builder();
            EgmPose.Builder pos = new EgmPose.Builder();
            EgmQuaternion.Builder pq = new EgmQuaternion.Builder();
            EgmCartesian.Builder pc = new EgmCartesian.Builder();

            pc.SetX(PoseQuat[0])
                .SetY(PoseQuat[1])
                .SetZ(PoseQuat[2]);

            pq.SetU0(PoseQuat[3])
                .SetU1(PoseQuat[4])
                .SetU2(PoseQuat[5])
                .SetU3(PoseQuat[6]);


            pos.SetPos(pc)
                .SetOrient(pq);

            planned.SetCartesian(pos);  // bind pos object to planned
            sensor.SetPlanned(planned); // bind planned to sensor object

            if (OptLinSpeed > 0 && OptOrientSpeed > 0) sensor.SetSpeedRef(speedbuilder);

            return;
        }

        private void CreateJointMessage(EgmSensor.Builder sensor)
        {
            // create a header
            EgmHeader.Builder hdr = new EgmHeader.Builder();
            hdr.SetSeqno(_SeqNumber++)
                .SetTm((uint)DateTime.Now.Ticks)
                .SetMtype(EgmHeader.Types.MessageType.MSGTYPE_CORRECTION);

            sensor.SetHeader(hdr);

            // create some sensor data
            EgmPlanned.Builder planned = new EgmPlanned.Builder();
            EgmJoints.Builder joint = new EgmJoints.Builder();

            joint.AddJoints(1);
            joint.AddJoints(2);
            joint.AddJoints(3);
            joint.AddJoints(4);
            joint.AddJoints(5);
            joint.AddJoints(6);

            joint.SetJoints(0, Joints[0])
                  .SetJoints(1, Joints[1])
                  .SetJoints(2, Joints[2])
                  .SetJoints(3, Joints[3])
                  .SetJoints(4, Joints[4])
                  .SetJoints(5, Joints[5]);

            planned.SetJoints(joint);
            sensor.SetPlanned(planned); // bind planned to sensor object

            return;
        }

        private void DisplayInboundMessage(EgmRobot robot)
        {
            if (robot.HasHeader && robot.Header.HasSeqno && robot.Header.HasTm)
            {
                //Console.WriteLine("Seq={0} tm={1}",
                //robot.Header.Seqno.ToString(), robot.Header.Tm.ToString());
                Console.WriteLine("Seq={0} tm={1}",
                    robot.FeedBack.Cartesian.Pos.X, robot.Header.Tm.ToString());
            }
            else
            {
                Console.WriteLine("No header in robot message");
            }
        }

        private void GetCurrentPos(byte[] data)
        {
            EgmRobot robot = EgmRobot.CreateBuilder().MergeFrom(data).Build();
            CurrentPose = new double[] {robot.FeedBack.Cartesian.Pos.X,
            robot.FeedBack.Cartesian.Pos.Y,
            robot.FeedBack.Cartesian.Pos.Z,
            robot.FeedBack.Cartesian.Orient.U0,
            robot.FeedBack.Cartesian.Orient.U1,
            robot.FeedBack.Cartesian.Orient.U2,
            robot.FeedBack.Cartesian.Orient.U3};
            
            RobotTime = robot.FeedBack.Time.Usec;


        }

        public void Start()
        {
            if (_SensorThread != null && _SensorThread.IsAlive)
            {
                _SensorThread.Abort();
            }
            _RunThread = true;
            _SensorThread = new Thread(new ThreadStart(UDPUCPoseThread));
            _SensorThread.Start();
        }

        // Stop and exit thread
        public void Stop()
        {
            _RunThread = false;
            if (_SensorThread != null)
            {
                _SensorThread.Interrupt();
                _SensorThread.Abort();
            }
            if (_UDPServer != null)
            {
                _UDPServer.Close();
            }

        }
    }

}
