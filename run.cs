﻿using OpenTK.Graphics.OpenGL;
using System;

namespace program
{
    public class OpenTKProgram
    {
        public static void Main()
        {
            main app = new main(1280, 720, "OpenGL");

            app.Initialize();

            ExecuteApp_Clear(app);

            app.Shutdown();
        }

        static void ExecuteApp_Clear(main app)
        {
            app.Run(() =>
            {
                GL.ClearColor(0.2f, 0.3f, 0.4f, 1.0f);
                GL.Clear(ClearBufferMask.ColorBufferBit);
            });

        }
    }
}
