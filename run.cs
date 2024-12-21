using OpenTK.Graphics.OpenGL;
using System;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Channels;

namespace PathFind3D
{
    // main app = new(1280, 720, "OpenGL");
    public class OpenTKProgram
    {
        public static void Main()
        {
            Console.WriteLine("run");
            using (var splashScreen = new splashScreen())
            {
                splashScreen.Run();
                main app = new(1280, 720, "OpenGL");
                Thread.Sleep(2000);
                splashScreen.Close();
                Thread.Sleep(200);
                app.Initialize();
                app.Run();
                app.Shutdown();
            }

        }
    }
}
