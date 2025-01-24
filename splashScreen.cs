using OpenTK.Mathematics;
using OpenTK.Windowing.Desktop;
using System;
using System.Diagnostics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;
using System.Threading;

namespace PathFind3D
{
    public class splashScreen : GameWindow
    {
        public splashScreen(int timeMillis = 3000) : base(GameWindowSettings.Default, new NativeWindowSettings
        {
            Title = "Splash Screen",
            ClientSize = (800, 600),
        })
        {
            maxFrame = (int)(60.0 * timeMillis / 1000.0);
            frame = 0;
        }

        private int frame = 0;
        private int maxFrame;

        protected override void OnLoad()
        {
            CenterWindow();
            base.OnLoad();
            GL.ClearColor(new Color4(100, 149, 237, 255)); // Background color
        }

        protected override void OnRenderFrame(FrameEventArgs args)
        {
            base.OnRenderFrame(args);
            GL.Clear(ClearBufferMask.ColorBufferBit);

            SwapBuffers();
        }

        protected override void OnUnload()
        {
            base.OnUnload();
        }

        protected override void OnUpdateFrame(FrameEventArgs args)
        {
            base.OnUpdateFrame(args);
            frame++;

            if (frame > maxFrame)
            {
                Close();
                return;
            }

            int targetFrameTime = 1000 / 60;
            int millisToWait = targetFrameTime - (int)(args.Time * 1000);

            if (millisToWait > 0)
            {
                Thread.Sleep(millisToWait);
            }
        }

    }
}
