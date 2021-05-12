using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace EGMWpf
{
    public class EGMVector3
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public EGMVector3(double x,double y,double z)
        {
            X = x;
            Y = y;
            Z = z;
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
}
