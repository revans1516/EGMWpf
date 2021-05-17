using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace EGMWpf
{
    public class EGMVector3
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public EGMVector3(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public EGMVector3(Vector3 Vec)
        {
            X = Vec.X;
            Y = Vec.Y;
            Z = Vec.Z;
        }

        public Vector3 ToVector3()
		{
            return new Vector3((float)X, (float)Y, (float)Z);
		}

    }
    public class EGMVector4
    {
        public double V0 { get; set; }
        public double V1 { get; set; }
        public double V2 { get; set; }
        public double V3 { get; set; }
        public EGMVector4(double v0, double v1, double v2, double v3)
        {
            V0 = v0;
            V1 = v1;
            V2 = v2;
            V3 = v3;
        }
    }
    public class EGMEulerPose
    {
        public EGMVector3 Position { get; set; }
        public EGMVector3 Rotation { get; set; }
        public EGMEulerPose(EGMVector3 _position, EGMVector3 _rotation)
        {
            Position = _position;
            Rotation = _rotation;
        }
        public EGMEulerPose(double x, double y, double z, double rx, double ry, double rz)
        {
            Position = new EGMVector3(x, y, z);
            Rotation = new EGMVector3(rx, ry, rz);
        }
    }
    public class EGMQuaternionPose
    {
        public EGMVector3 Position { get; set; }
        public EGMVector4 Rotation { get; set; }
        public EGMQuaternionPose(EGMVector3 _position, EGMVector4 _rotation)
        {
            Position = _position;
            Rotation = _rotation;
        }
        public EGMQuaternionPose(double x, double y, double z, double rw, double rx, double ry, double rz)
        {
            Position = new EGMVector3(x,y,z);
            Rotation = new EGMVector4(rw,rx,ry, rz);
        }
    }
}
