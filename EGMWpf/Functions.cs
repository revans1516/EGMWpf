using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;

namespace EGMWpf
{
    static class Functions
    {
        /// <summary>
        /// Convert a temperature from Celsius to Fahrenheit 
        /// </summary>
        public static double Celsius2Fahrenheit(double temp)
        {
            return (temp * (9 / 5)) + 32;
        }

        /// <summary>
        /// Convert an array of doubles from Celsius to Fahrenheit
        /// </summary>
        public static double[] Celsius2Fahrenheit(double[] temp)
        {
            double[] ret = new double[temp.Length];
            int count = 0;

            foreach (double d in temp)
            {
                ret[count] = Celsius2Fahrenheit(d);
                count++;
            }

            return ret;
        }

        /// <summary>
        /// Convert a Fahrenheit tempature to Celsius
        /// </summary>
        public static double Fahrenheit2Celsius(double temp)
        {
            return (temp - 32) * 5 / 9;
        }

        /// <summary>
        /// Reverse a string
        /// </summary>
        public static string Reverse(string s)
        {
            char[] ca = s.ToCharArray();
            Array.Reverse(ca);
            return new string(ca);
        }

        /// <summary>
        /// Check to see if the actual and target are within tolerance.
        /// </summary>
        public static bool InTolerance(double Actual, double Target, double Tolerance)
        {
            return Math.Abs(Actual - Target) < Tolerance;
        }

        /// <summary>
        /// Get the viapoint from two points and a distance.
        /// </summary>
        public static double[] Via_Point(double[] P1, double[] P2, double Distance)
        {
            double det;
            double slope;
            double[] unit;
            var P3 = new double[2];

            slope = (P2[1] - P1[1]) / (P2[0] - P1[0]);
            det = 1d / slope + slope;
            P3[0] = -(P1[1] - slope * P1[0]) / det;
            P3[1] = (P1[1] / slope - P1[0]) / det;

            if (slope == 0d)
            {
                P3[1] = P1[1];
                unit = new[] { 0d, -1 };
            }
            else if (double.IsInfinity(slope))
            {
                P3[0] = P1[0];
                unit = new[] { 1d, 0d };
            }
            else
            {
                unit = new[] { 1d, -1 / slope };
                unit = new[] { unit[0] / Math.Sqrt(Math.Pow(unit[0], 2d) + Math.Pow(unit[1], 2d)), unit[1] / Math.Sqrt(Math.Pow(unit[1], 2d) + Math.Pow(unit[1], 2d)) };
            }

            while (Math.Abs(P3[0] * P1[1] - P3[1] * P1[0]) / Math.Sqrt(Math.Pow(P3[1] - P1[1], 2d) + Math.Pow(P3[0] - P1[0], 2d)) < Distance || Math.Abs(P3[0] * P2[1] - P3[1] * P2[0]) / Math.Sqrt(Math.Pow(P3[1] - P2[1], 2d) + Math.Pow(P3[0] - P2[0], 2d)) < Distance)
            {
                P3[0] = P3[0] + 0.005d * unit[0];
                P3[1] = P3[1] + 0.005d * unit[1];
            }

            return P3;
        }

        /// <summary>
        /// Project a point onto a line.
        /// </summary>
        public static double[] Get_Projected_Point_on_line(double[] P1, double[] P2, double[] PointToProject)
        {
            var e1 = new decimal[2];
            var e2 = new decimal[2];
            var P1Dec = new decimal[2];
            var P2Dec = new decimal[2];
            var PReturn = new double[2];

            e1[0] = (decimal)(P2[0] - P1[0]);
            e1[1] = (decimal)(P2[1] - P1[1]);

            e2[0] = (decimal)(PointToProject[0] - P1[0]);
            e2[1] = (decimal)(PointToProject[1] - P1[1]);

            foreach (var i in new[] { 0, 1 })
            {
                P1Dec[i] = Convert.ToDecimal(P1[i]);
                P2Dec[i] = Convert.ToDecimal(P2[i]);
            }

            double DotP = decimal.ToDouble(DotProduct(P1Dec, P2Dec));
            double Len = 0d;

            e1.ToList().ForEach(x => Len += Math.Pow((double)x, 2d));

            PReturn[0] = P1[0] + DotP * (double)e1[0] / Len;
            PReturn[1] = P1[1] + DotP * (double)e1[1] / Len;

            return PReturn;
        }

        /// <summary>
        /// Make sure this works the way we want it..
        /// </summary>
        public static decimal DotProduct(decimal[] P1, decimal[] P2)
        {
            return Enumerable.Zip<decimal, decimal, decimal>((IEnumerable<decimal>)P1, (IEnumerable<decimal>)P2, (x, y) => x * y).Sum();
        }

        /// <summary>
        /// returns true/false if the points are between each other.
        /// </summary>
        public static bool Between(double Value, double Value1, double Value2, bool Inclusive = false)
        {
            double Upper;
            double Lower;

            if (Value1 > Value2)
            {
                Upper = Value1;
                Lower = Value2;
            }
            else
            {
                Lower = Value1;
                Upper = Value2;
            }

            if (Inclusive)
            {
                return Upper >= Value & Value >= Lower;
            }
            else
            {
                return Upper > Value & Value > Lower;
            }
        }

        /// <summary>
        /// returns true/false if the points are between each other.
        /// </summary>
        public static bool Between(int Value, int Value1, int Value2, bool Inclusive = false)
        {
            int Upper;
            int Lower;

            if (Value1 > Value2)
            {
                Upper = Value1;
                Lower = Value2;
            }
            else
            {
                Lower = Value1;
                Upper = Value2;
            }

            if (Inclusive)
            {
                return Upper >= Value & Value >= Lower;
            }
            else
            {
                return Upper > Value & Value > Lower;
            }
        }

        /// <summary>
        /// Transform Bytes into an in.
        /// </summary>
        public static int Byte2Int(byte[] Bytes)
        {
            try
            {
                return BitConverter.ToInt32(CheckBytes(Bytes), 0);
            }
            catch (Exception)
            {
                return 0;
            }
        }

        /// <summary>
        /// Interpolate some data from the given points.
        /// </summary>
        private static double Interpolate(double a1, double a2, double b1, double b2, double c1)
        {
            // *------------+-----*
            // a               c      b
            // 1               3     5
            // 10             x      50

            return a2 + (b2 - a2) * (c1 - a1) / (b1 - a1);
        }

        /// <summary>
        /// Interpolate a path from a target and two points.
        /// </summary>
        public static double Interpolate(double[] Target, double[] P1, double[] P2)
        {
            double[] Upper;
            double[] Lower;

            if (double.IsNaN(Target[0]))
            {
                if (P1[0] > P2[0])
                {
                    Lower = P2;
                    Upper = P1;
                }
                else
                {
                    Lower = P1;
                    Upper = P2;
                }

                return Interpolate(Lower[1], Lower[0], Upper[1], Upper[0], Target[1]);
            }
            else
            {
                if (P1[1] > P2[1])
                {
                    Lower = P2;
                    Upper = P1;
                }
                else
                {
                    Lower = P1;
                    Upper = P2;
                }

                return Interpolate(Lower[0], Lower[1], Upper[0], Upper[1], Target[0]);
            }
        }

        /// <summary>
        /// Get the X from the line.
        /// </summary>
        public static double X_From_Line(double Y, double Slope, double YIntercept)
        {
            return (Y - YIntercept) / Slope;
        }

        /// <summary>
        /// Get the Y from the line.
        /// </summary>
        public static double Y_From_Line(double X, double Slope, double YIntercept)
        {
            return Slope * X + YIntercept;
        }

        /// <summary>
        /// Calculate the slope
        /// </summary>
        public static double Slope(Point P1, Point P2)
        {
            double DeltaX = P2.X - P1.X;
            double DeltaY = P2.Y - P1.Y;

            if (DeltaX == 0d)
            {
                return double.PositiveInfinity * Math.Sign(DeltaY);
            }
            else
            {
                return DeltaY / DeltaX;
            }
        }

        /// <summary>
        /// Calculate the angles between the two points.
        /// </summary>
        public static double Angle(Point P1, Point P2)
        {
            double DeltaX = P2.X - P1.X;
            double DeltaY = P2.Y - P1.Y;
            if (P2.Y < P1.Y & Math.Abs(DeltaX) < 0.005d)
            {
                return 0d;
            }
            else if (P2.Y > P1.Y & Math.Abs(DeltaX) < 0.005d)
            {
                return Math.PI;
            }
            else if (P2.X > P1.X & Math.Abs(DeltaY) < 0.005d)
            {
                return Math.PI / 2d;
            }
            else if (P2.X < P1.X & Math.Abs(DeltaY) < 0.005d)
            {
                return -Math.PI / 2d;
            }
            else if (DeltaY < 0d & DeltaX > 0d)
            {
                return -Math.Atan(DeltaX / DeltaY);
            }
            else if (DeltaY > 0d & DeltaX < 0d)
            {
                return Math.Atan(DeltaX / DeltaY) - Math.PI / 2d;
            }
            else if (DeltaY > 0d & DeltaX > 0d)
            {
                return Math.Atan(DeltaX / DeltaY) + Math.PI / 2d;
            }
            else // If DeltaY > 0 And DeltaX < 0 Then
            {
                return -Math.Atan(DeltaX / DeltaY);
            }
        }

        /// <summary>
        /// Calculate the intercept from a double array.
        /// </summary>
        public static double Intercept(double[] Point, double Slope)
        {
            return Point[1] - Slope * Point[0];
        }

        /// <summary>
        /// Calculate the intercept from a point.
        /// </summary>
        public static double Intercept(Point Point, double Slope)
        {
            return Point.Y - Slope * Point.X;
        }

        /// <summary>
        /// Convert a byte to an int.
        /// </summary>
        public static uint Byte2UInt(byte[] Bytes)
        {
            try
            {
                return BitConverter.ToUInt32(CheckBytes(Bytes), 0);
            }
            catch (Exception)
            {
                return 0U;
            }
        }

        /// <summary>
        /// Convert a byte to an int64(ulong).
        /// </summary>
        public static ulong Byte2UInt64(byte[] Bytes)
        {
            try
            {
                return BitConverter.ToUInt64(CheckBytes(Bytes), 0);
            }
            catch (Exception)
            {
                return 0UL;
            }
        }

        /// <summary>
        /// Convert a byte to a char.
        /// </summary>
        public static char Byte2Char(byte[] Bytes)
        {
            try
            {
                return BitConverter.ToChar(CheckBytes(Bytes), 0);
            }
            catch (Exception)
            {
                return Convert.ToChar("");
            }
        }

        /// <summary>
        /// Change a byte array into a double.
        /// </summary>
        public static double Byte2Double(byte[] Bytes)
        {
            try
            {
                return BitConverter.ToDouble(CheckBytes(Bytes), 0);
            }
            catch (Exception)
            {
                return double.NaN;
            }
        }

        /// <summary>
        /// Check to make sure the bytes are in the right format.
        /// </summary>
        private static byte[] CheckBytes(byte[] Bytes)
        {
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(Bytes);
            }

            return Bytes;
        }

        /// <summary>
        /// Calc rads to deg return as decimal.
        /// </summary>
        public static decimal Radians2Degrees(decimal Radians)
        {
            return (decimal)((double)Radians * (180d / Math.PI));
        }

        /// <summary>
        /// Calc deg to rads return as decimal.
        /// </summary>
        public static decimal Degrees2Radians(decimal Degrees)
        {
            return (decimal)((double)Degrees * (Math.PI / 180d));
        }

        /// <summary>
        /// Convert inches to mm
        /// </summary>
        public static decimal Inches2MM(decimal Inches)
        {
            return Inches * 25.4m;
        }

        public static decimal Meters2MM(decimal Meters)
        {
            return Meters * 1000m;
        }

        /// <summary>
        /// Convert mm to M.
        /// </summary>
        public static decimal MM2Meters(decimal MM)
        {
            return MM / 1000m;
        }
        /// <summary>
        /// convert a point that is in mm to a point that is in m
        /// </summary>
        public static Point MMPoint2Meters(Point MMPoint)
        {
            return new Point(MM2Meters(MMPoint.X), MM2Meters(MMPoint.Y));
        }

        /// <summary>
        /// Convert from mm to inches.
        /// </summary>
        public static decimal MM2Inches(decimal MM)
        {
            return MM / 25.4m;
        }

        /// <summary>
        /// Change from radians and go to degrees.
        /// </summary>
        public static double Radians2Degrees(double Radians)
        {
            return Radians * (180.0d / Math.PI);
        }

        /// <summary>
        /// Go from degrees to radians.
        /// </summary>s
        public static double Degrees2Radians(double Degrees)
        {
            return Degrees * (Math.PI / 180.0d);
        }

        public static double[] Radians2Degrees(double[] arr)
        {
            int count = 0;
            foreach (double d in arr)
            {
                arr[count] = d * 180 / Math.PI;
                count++;
            }

            return arr;
        }

        /// <summary>
        /// Convert from inches to mm
        /// </summary>
        public static double Inches2MM(double Inches)
        {
            return Inches * 25.4d;
        }

        /// <summary>
        /// convert from m to mm.
        /// </summary>
        public static double Meters2MM(double Meters)
        {
            return Meters * 1000.0d;
        }

        /// <summary>
        /// Change from millimeters to meters.
        /// </summary>
        public static double MM2Meters(double MM)
        {
            return MM / 1000.0d;
        }

        /// <summary>
        /// Convert from mm to inches.
        /// </summary>
        public static double MM2Inches(double MM)
        {
            return MM / 25.4d;
        }

        /// <summary>
        /// Calculate the Magnitude of some numbers. Return a decimal.
        /// </summary>
        public static decimal Magnitude(List<decimal> Numbers)
        {
            decimal tmpNum = 0m;
            Numbers.ForEach(x => tmpNum = (decimal)(tmpNum + Convert.ToDecimal(Math.Pow((double)x, 2d))));

            return (decimal)Math.Sqrt((double)tmpNum);
        }

        /// <summary>
        /// Calculate the magnitude and return as decimal.
        /// </summary>
        public static decimal Magnitude(decimal[] Numbers)
        {
            return Magnitude(Numbers.ToList());
        }

        /// <summary>
        /// Do what we do for the other magnitude function. Return a double.
        /// </summary>
        public static double Magnitude(double[] Numbers)
        {
            return Magnitude(Numbers.ToList());
        }

        /// <summary>
        /// Calculate the magnitude and then return a double.
        /// </summary>
        public static double Magnitude(List<double> Numbers)
        {
            double tmpNum = 0d;
            Numbers.ForEach(x => tmpNum += Math.Pow(x, 2d));

            return Math.Sqrt(tmpNum);
        }

        /// <summary>
        /// Calculate the magnitude of the point. Return a double.
        /// </summary>
        /// <returns></returns>
        public static double Magnitude(Point Pt)
        {
            return Math.Sqrt(Math.Pow(Pt.X, 2) + Math.Pow(Pt.Y, 2));
        }


        /// <summary>
        /// Distance from the line made between Point 1 and Point 2 to the Target
        /// Only using first two (X,Y) values
        /// </summary>
        public static double Distance(double[] Point1, double[] Point2, double[] Target)
        {
            double X0 = Target[0];
            double Y0 = Target[1];
            double X1 = Point1[0];
            double Y1 = Point1[1];
            double X2 = Point2[0];
            double Y2 = Point2[1];

            return Math.Abs((Y2 - Y1) * X0 - (X2 - X1) * Y0 + X2 * Y1 - Y2 * X1) / Math.Sqrt(Math.Pow(Y2 - Y1, 2d) + Math.Pow(X2 - X1, 2d));
        }

        /// <summary>
        /// Calculate the distance between 3 points.
        /// </summary>
        public static double Distance(Point Point1, Point Point2, Point Point3)
        {
            return Distance(new[] { Point1.X, Point1.Y }, new[] { Point2.X, Point2.Y }, new[] { Point3.X, Point3.Y });
        }

        /// <summary>
        /// Distance from the line made between Point 1 and Point 2 to the Target
        /// Only using first two (X,Y) values
        /// </summary>
        public static double Distance(Point Point1, Point Point2)
        {
            return Distance(new[] { Point1.X, Point1.Y }, new[] { Point2.X, Point2.Y });
        }

        /// <summary>
        /// Distance between two points. Can be 2D or 3D coordinates.
        /// </summary>
        public static double Distance(double[] Point1, double[] Point2)
        {
            return Math.Sqrt(Math.Pow(Point2[0] - Point1[0], 2d) + Math.Pow(Point2[1] - Point1[1], 2d));
        }

        /// <summary>
        /// Normalize two points to each other.
        /// </summary>
        public static Point Normalize(Point Point1, Point Point2)
        {
            return new Point((Point2.X - Point1.X) / Length_Line(Point1, Point2), (Point2.Y - Point1.Y) / Length_Line(Point1, Point2));
        }

        /// <summary>
        /// Return the length of a line.
        /// </summary>
        public static double Length_Line(Point Point1, Point Point2)
        {
            return Distance(Point1, Point2);
        }

        /// <summary>
        /// Calculate the length of a line from two arrays
        /// </summary>
        public static double Length_Line(double[] Point1, double[] Point2)
        {
            return Distance(Point1, Point2);
        }

        /// <summary>
        /// Get a point that is a long the line between two points.
        /// </summary>
        public static Point GetPointAlongLineAtDistance(Point Point1, Point Point2, double DistanceFromPoint1)
        {
            var V = new Point(Point2.X - Point1.X, Point2.Y - Point1.Y);

            return new Point(Point1.X + DistanceFromPoint1 * V.X / Magnitude(V), Point1.Y + DistanceFromPoint1 * V.Y / Magnitude(V));
        }

        /// <summary>
        /// Get the offset from a point line.
        /// </summary>
        public static List<double[]> GetOffsetPointFromLine(double[] Point1, double[] Point2, double[] Point3, double Distance)
        {
            var NewPoints = new List<double[]>();
            List<Point> RetPoints = GetOffsetPointFromLine(new Point(Point1[0], Point1[1]), new Point(Point2[0], Point2[1]), new Point(Point3[0], Point3[1]), Distance);
            RetPoints.ForEach(Point => NewPoints.Add(Point2Double(Point)));

            return NewPoints;
        }

        /// <summary>
        /// Transform a point into an array of doubles.
        /// </summary>
        public static double[] Point2Double(Point Point)
        {
            return new[] { Point.X, Point.Y };
        }

        /// <summary>
        /// Convert a double to a point.
        /// </summary>
        public static Point Double2Point(double[] DoublePoint)
        {
            return new Point(DoublePoint[0], DoublePoint[1]);
        }

        /// <summary>
        /// Get the offset from the line.
        /// </summary>
        public static List<Point> GetOffsetPointFromLine(Point Point1, Point Point2, Point Point3, double Distance)
        {
            double m = Slope(Point1, Point2);
            m = -1 / m;

            var NewPointPlus = default(Point);
            var NewPointMinus = default(Point);

            NewPointPlus.X = Point3.X + Distance * Math.Sqrt(1d / (1d + Math.Pow(m, 2d)));
            NewPointPlus.Y = Point3.Y + m * Distance * Math.Sqrt(1d / (1d + Math.Pow(m, 2d)));
            NewPointMinus.X = Point3.X - Distance * Math.Sqrt(1d / (1d + Math.Pow(m, 2d)));
            NewPointMinus.Y = Point3.Y - m * Distance * Math.Sqrt(1d / (1d + Math.Pow(m, 2d)));

            var NewPoints = new List<Point>() { NewPointPlus, NewPointMinus };

            return NewPoints;
        }

        /// <summary>
        /// Calculate the length of an arc.
        /// </summary>
        private static double Length_Arc(double[] Point1, double[] Point2, double[] Point3)
        {
            double Det1;
            double Det2;
            double Det3;
            double Det4;
            double CenterX;
            double CenterY;
            double Radius;
            double Angle;

            // Do some math
            var Mat1 = new double[3, 3] { { Point1[0], Point1[1], 1.0d }, { Point2[0], Point2[1], 1.0d }, { Point3[0], Point3[1], 1.0d } };
            var Mat2 = new double[3, 3] { { Math.Pow(Point1[0], 2d) + Math.Pow(Point1[1], 2d), Point1[1], 1.0d }, { Math.Pow(Point2[0], 2d) + Math.Pow(Point2[1], 2d), Point2[1], 1.0d }, { Math.Pow(Point3[0], 2d) + Math.Pow(Point3[1], 2d), Point3[1], 1.0d } };
            var Mat3 = new double[3, 3] { { Math.Pow(Point1[0], 2d) + Math.Pow(Point1[1], 2d), Point1[0], 1.0d }, { Math.Pow(Point2[0], 2d) + Math.Pow(Point2[1], 2d), Point2[0], 1.0d }, { Math.Pow(Point3[0], 2d) + Math.Pow(Point3[1], 2d), Point3[0], 1.0d } };
            var Mat4 = new double[3, 3] { { Math.Pow(Point1[0], 2d) + Math.Pow(Point1[1], 2d), Point1[0], Point1[1] }, { Math.Pow(Point2[0], 2d) + Math.Pow(Point2[1], 2d), Point2[0], Point2[1] }, { Math.Pow(Point3[0], 2d) + Math.Pow(Point3[1], 2d), Point3[0], Point3[1] } };

            Det1 = Mat1[0, 0] * Mat1[1, 1] * Mat1[2, 2] + Mat1[0, 1] * Mat1[1, 2] * Mat1[2, 0] + Mat1[0, 2] * Mat1[1, 0] * Mat1[2, 1] - (Mat1[2, 0] * Mat1[1, 1] * Mat1[0, 2] + Mat1[2, 1] * Mat1[1, 2] * Mat1[0, 0] + Mat1[2, 2] * Mat1[1, 0] * Mat1[0, 1]);
            Det2 = Mat2[0, 0] * Mat2[1, 1] * Mat2[2, 2] + Mat2[0, 1] * Mat2[1, 2] * Mat2[2, 0] + Mat2[0, 2] * Mat2[1, 0] * Mat2[2, 1] - (Mat2[2, 0] * Mat2[1, 1] * Mat2[0, 2] + Mat2[2, 1] * Mat2[1, 2] * Mat2[0, 0] + Mat2[2, 2] * Mat2[1, 0] * Mat2[0, 1]);
            Det3 = Mat3[0, 0] * Mat3[1, 1] * Mat3[2, 2] + Mat3[0, 1] * Mat3[1, 2] * Mat3[2, 0] + Mat3[0, 2] * Mat3[1, 0] * Mat3[2, 1] - (Mat3[2, 0] * Mat3[1, 1] * Mat3[0, 2] + Mat3[2, 1] * Mat3[1, 2] * Mat3[0, 0] + Mat3[2, 2] * Mat3[1, 0] * Mat3[0, 1]);
            Det4 = Mat4[0, 0] * Mat4[1, 1] * Mat4[2, 2] + Mat4[0, 1] * Mat4[1, 2] * Mat4[2, 0] + Mat4[0, 2] * Mat4[1, 0] * Mat4[2, 1] - (Mat4[2, 0] * Mat4[1, 1] * Mat4[0, 2] + Mat4[2, 1] * Mat4[1, 2] * Mat4[0, 0] + Mat4[2, 2] * Mat4[1, 0] * Mat4[0, 1]);

            CenterX = Det2 / (2d * Det1);
            CenterY = -Det3 / (2d * Det1);

            Radius = Math.Sqrt((Math.Pow(Det2, 2d) + Math.Pow(Det3, 2d) + 4d * Det1 * Det4) / (4d * Math.Pow(Det1, 2d)));

            Angle = Math.Acos(((Point1[0] - CenterX) * (Point2[0] - CenterX) + (Point1[1] - CenterY) * (Point2[1] - CenterY)) / (Math.Sqrt(Math.Pow(Point1[0] - CenterX, 2d) + Math.Pow(Point1[1] - CenterY, 2d)) * Math.Sqrt(Math.Pow(Point2[0] - CenterX, 2d) + Math.Pow(Point2[1] - CenterY, 2d))));
            Angle += Math.Acos(((Point2[0] - CenterX) * (Point3[0] - CenterX) + (Point2[1] - CenterY) * (Point3[1] - CenterY)) / (Math.Sqrt(Math.Pow(Point2[0] - CenterX, 2d) + Math.Pow(Point2[1] - CenterY, 2d)) * Math.Sqrt(Math.Pow(Point3[0] - CenterX, 2d) + Math.Pow(Point3[1] - CenterY, 2d))));

            return Radius * Angle;
        }


        /// <summary>
        /// Convert a list of numbers provided as a string to an array of values
        /// </summary>
        public static double[] String_2_Doubles(string strvalues)
        {
            return String_2_Doubles(strvalues.Split(','));
        }

        /// <summary>
        /// Try and transform a string array into a double array.
        /// </summary>
        public static double[] String_2_Doubles(string[] StringArray)
        {
            return Array.ConvertAll(StringArray, new Converter<string, double>(double.Parse));
        }

        /// <summary>
        /// Convert an array of values as a string
        /// </summary>
        public static string Doubles_2_String(double[] DoublesArray)
        {
            return string.Join(",", Doubles_2_StringArray(DoublesArray));
        }

        /// <summary>
        /// Convert a double array to a string
        /// </summary>
        public static string Doubles_2_String(double[] DoublesArray, int Digits)
        {
            return Doubles_2_String(RoundArray(DoublesArray, Digits));
        }

        /// <summary>
        /// Round all the digits in the array.
        /// </summary>
        public static double[] RoundArray(double[] Arry, int Digits)
        {
            for (int i = 0, loopTo = Arry.Length - 1; i <= loopTo; i++)
            {
                Arry[i] = Math.Round(Arry[i], Digits);
            }

            return Arry;
        }

        /// <summary>
        /// Convert a double array into a string array.
        /// </summary>
        public static string[] Doubles_2_StringArray(double[] DoublesArray)
        {
            return Array.ConvertAll(DoublesArray, new Converter<double, string>(Convert.ToString));
        }

        /// <summary>
        /// Convert a doubles array to a string array rounded to a specific digit.
        /// </summary>
        public static string[] Doubles_2_StringArray(double[] DoublesArray, int Digits)
        {
            return Array.ConvertAll(RoundArray(DoublesArray, Digits), new Converter<double, string>(Convert.ToString));
        }

        /// <summary>
        /// Check to see if two arrays are equal.
        /// </summary>
        public static bool EqualArrays(bool[] Array1, bool[] Array2)
        {
            if (Array1.Length != Array2.Length)
            {
                return false;
            }

            for (int i = 0, loopTo = Array1.Length - 1; i <= loopTo; i++)
            {
                if (Array1[i] != Array2[i])
                {
                    return false;
                }
            }

            return true;
        }
        /// <summary>
        /// Scale the target value to be within 255.
        /// </summary>
        public static short ScaleTo255(short Target, short Range = 100)
        {
            if (Target > Range)
            {
                Target = Range;
            }

            if (Target < 0)
            {
                Target = 0;
            }

            return (short)(255 * Target / (double)Range);
        }
    }
}
