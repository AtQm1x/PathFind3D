using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFind3D
{
    internal class MtH
    {
        public static double cubeRoot(double n)
        {
            return (System.Math.Pow(n, 1d / 3d));
        }

        public static float cubeRootF(float n)
        {
            return (System.MathF.Pow(n, 1f / 3f));
        }
    }
}
