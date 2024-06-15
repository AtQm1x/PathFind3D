using System;
using System.Drawing;
using System.Xml.Linq;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Common.Input;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTKBase;
using PathFind3D;

namespace program
{
    public class main
    {
        private int resX = 512;
        private int resY = 512;
        private string title = "App";
        private Action? runAction = null;
        private GameWindow? window = null;
        private float aspectRatio = 16f / 9;
        private float nodeDistance = 0.125f;
        Random rng = new();

        const int gridSize = 16;

        private bool exit = false;

        private Matrix4 projectionMatrix;
        private Matrix4 viewMatrix;
        private float zoom = 1.0f;
        private Vector3 cameraPosition = new Vector3(0, 0, 1);

        public main(int resX, int resY, string title)
        {
            this.resX = resX;
            this.resY = resY;
            this.title = title;
        }

        public bool Initialize()
        {
            window = new GameWindow(
                GameWindowSettings.Default,
                new NativeWindowSettings
                {
                    Size = (resX, resY),
                    Title = title,
                    Profile = ContextProfile.Compatability,
                    API = ContextAPI.OpenGL,
                    Flags = ContextFlags.Default,
                    Vsync = VSyncMode.Adaptive
                }
            );

            window.RenderFrame += onRenderFrame;
            window.UpdateFrame += onUpdateFrame;
            window.Resize += onResize;
            window.MouseWheel += onMouseWheel;
            window.MouseMove += onMouseMove;

            GL.Viewport(0, 0, resX, resY);

            // depth testing
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Less);

            // alpha blending
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);

            SetupMatrices();

            return true;
        }

        private void SetupMatrices()
        {
            float fov = MathHelper.DegreesToRadians(45.0f);
            projectionMatrix = Matrix4.CreatePerspectiveFieldOfView(fov, aspectRatio, 0.1f, 100.0f);
            UpdateViewMatrix();
        }

        private void UpdateViewMatrix()
        {
            viewMatrix = Matrix4.LookAt(
                cameraPosition,
                (cameraPosition.X, cameraPosition.Y, cameraPosition.Z - 1),
                Vector3.UnitY
            );
        }

        private void onResize(ResizeEventArgs args)
        {
            if (window == null)
                return;

            aspectRatio = (float)args.Width / args.Height;
            GL.Viewport(0, 0, args.Width, args.Height);

            float fov = MathHelper.DegreesToRadians(45.0f);
            projectionMatrix = Matrix4.CreatePerspectiveFieldOfView(fov, aspectRatio, 0.1f, 100.0f);
        }

        public void Shutdown() { }

        GraphNode[,,] graph;

        Vector3 rot = new(0);

        private void rebuildGrid()
        {
            graph = new GraphNode[gridSize, gridSize, gridSize];
            float startOffset = ((int)MtH.cubeRoot(graph.Length) - 1) * nodeDistance / 2;

            int index = 0;
            for (int i = 0; i < MtH.cubeRoot(graph.Length); i++)
            {
                for (int j = 0; j < MtH.cubeRoot(graph.Length); j++)
                {
                    for (int k = 0; k < MtH.cubeRoot(graph.Length); k++)
                    {
                        float posX = i * nodeDistance - startOffset;
                        float posY = j * nodeDistance - startOffset;
                        float posZ = k * nodeDistance - startOffset;
                        GraphNode nd = new GraphNode(posX, posY, posZ);
                        if (rng.NextDouble() >= 0.05)
                        {
                            nd.DrawMD = GraphNode.DrawMode.None;
                        }
                        else
                        {
                            nd.DrawMD = GraphNode.DrawMode.Both;
                        }
                        graph[index++] = nd;
                    }
                }
            }
        }

        public void Run(Action mainLoopFunction)
        {
            runAction = mainLoopFunction;
            rebuildGrid();

            window?.Run();
        }

        private void onUpdateFrame(FrameEventArgs e)
        {
            if (window == null)
                return;

            if (window.KeyboardState.IsKeyDown(Keys.Escape))
            {
                window.Close();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.R))
            {
                rebuildGrid();
            }

            foreach (GraphNode node in graph)
            {
                node.Rotation = rot;
                node.AspectRatio = aspectRatio;
                node.updateCenter(viewMatrix, projectionMatrix);
            }


            runAction?.Invoke();
        }

        private void onRenderFrame(FrameEventArgs args)
        {
            if (window == null)
                return;

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            foreach (GraphNode node in graph)
            {
                node.Draw(viewMatrix, projectionMatrix);
            }

            var nd = new GraphNode(0.125f / 2, 0.125f / 2, 0.125f / 2);
            nd.BaseSize = MtH.cubeRootF(graph.Length) * 0.125f + 0.01f;
            nd.Rotation = rot;
            nd.AspectRatio = aspectRatio;
            nd.updateCenter(viewMatrix, projectionMatrix);
            nd.DrawMD = GraphNode.DrawMode.Wireframe;
            nd.Draw(viewMatrix, projectionMatrix);
            window?.SwapBuffers();
        }

        private void onMouseWheel(MouseWheelEventArgs e)
        {
            zoom -= e.OffsetY * 0.1f;
            zoom = Math.Max(0.1f, Math.Min(zoom, 20.0f));
            cameraPosition.Z = zoom;
            UpdateViewMatrix();
        }

        private void onMouseMove(MouseMoveEventArgs e)
        {
            if (window == null)
                return;

            if (window.MouseState.IsButtonDown(MouseButton.Left))
            {
                rot.Y += (window.MouseState.Delta.X / 100f);
                rot.X += (window.MouseState.Delta.Y / 100f);

                rot.X = Math.Max(-1.5f, Math.Min(1.5f, rot.X));
            }

            if (window.MouseState.IsButtonDown(MouseButton.Middle))
            {
                cameraPosition.X -= window.MouseState.Delta.X / 300f;
                cameraPosition.Y += window.MouseState.Delta.Y / 300f;

                UpdateViewMatrix();
            }
        }
    }
}
