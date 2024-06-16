using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTKBase;
using PathFind3D;
using System;
using System.Threading;
using static OpenTKBase.GraphNode;

namespace program
{
    public class main
    {
        private int resX;
        private int resY;
        private string title;
        private Action? runAction = null;
        private GameWindow? window = null;
        private float aspectRatio = 16f / 9;
        private float nodeDistance = 0.125f;
        private Random rng = new();
        private static Vector3i gridSize = new(64);
        private bool exit = false;
        private Matrix4 projectionMatrix;
        private Matrix4 viewMatrix;
        private float zoom = 1.0f;
        private Vector3 cameraPosition = new Vector3(0, 0, 1);
        private GraphNode[,,] grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];
        private Vector3 rot = new(0);
        private double nodeDensity = 0.005;

        //fps calculation stuff
        private double elapsedTime = 0.0;
        private int frameCount = 0;

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

            rebuildGrid();
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

        public void Shutdown()
        {
            exit = true;
        }

        private void rebuildGrid()
        {
            Thread thread = new Thread(() =>
            {
                runRebuildGridThread(ref grid);
            });

            thread.Start();
        }

        private void runRebuildGridThread(ref GraphNode[,,] grid)
        {
            if (grid == null)
                grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];

            float startOffset = ((int)MtH.cubeRoot(gridSize.X * gridSize.Y * gridSize.Z) - 1) * nodeDistance / 2;

            for (int i = 0; i < gridSize.X; i++)
            {
                for (int j = 0; j < gridSize.Y; j++)
                {
                    for (int k = 0; k < gridSize.Z; k++)
                    {
                        float posX = i * nodeDistance - startOffset;
                        float posY = j * nodeDistance - startOffset;
                        float posZ = k * nodeDistance - startOffset;
                        GraphNode nd = new(posX, posY, posZ)
                        {
                            Rotation = rot,
                            AspectRatio = aspectRatio
                        };
                        if (rng.NextDouble() >= nodeDensity)
                            nd.DrawMD = DrawMode.None;

                        else
                            nd.DrawMD = DrawMode.Both;

                        if (i == 0 && j == 0 && k == 0)
                            nd.DrawMD = DrawMode.Start;

                        if (i == gridSize.X - 1 && j == gridSize.Y - 1 && k == gridSize.Z - 1)
                            nd.DrawMD = DrawMode.End;

                        grid[i, j, k] = nd;
                    }
                }
            }
        }

        public void Run(Action mainLoopFunction)
        {
            runAction = mainLoopFunction;

            window?.Run();
        }
        private void onUpdateFrame(FrameEventArgs e)
        {
            if (window == null)
                return;

            foreach (GraphNode node in grid)
            {
                if (node != null && node.DrawMD != DrawMode.None)
                {
                    if (node.Rotation != rot)
                        node.Rotation = rot;

                    if (node.AspectRatio != aspectRatio)
                        node.AspectRatio = aspectRatio;
                }
            }

            elapsedTime += e.Time;
            frameCount++;

            if (elapsedTime >= 1.0)
            {
                double fps = frameCount / elapsedTime;
                window.Title = $"{title} - FPS: {fps:F2}";
                elapsedTime = 0.0;
                frameCount = 0;
            }


            if (window.KeyboardState.IsKeyDown(Keys.Escape))
            {
                exit = true;
                window.Close();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.R))
            {
                rebuildGrid();
            }

            runAction?.Invoke();
        }

        private void onRenderFrame(FrameEventArgs args)
        {
            if (window == null || grid == null)
                return;

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            drawGrid();

            GraphNode boxNode = new(0.125f / 2, 0.125f / 2, 0.125f / 2)
            {
                BaseSize = MtH.cubeRootF(gridSize.X * gridSize.Y * gridSize.Z) * 0.125f + 0.01f,
                Rotation = rot,
                AspectRatio = aspectRatio,
                DrawMD = DrawMode.Wireframe
            };
            boxNode.Draw(viewMatrix, projectionMatrix);

            window?.SwapBuffers();
        }

        private void drawGrid()
        {
            foreach (GraphNode node in grid)
            {
                if (node == null) continue;
                if (node.DrawMD == DrawMode.None) continue;
                node.Draw(viewMatrix, projectionMatrix);
            }
        }

        private void onMouseWheel(MouseWheelEventArgs e)
        {
            zoom -= e.OffsetY * 0.2f;
            zoom = Math.Max(0.1f, Math.Min(zoom, 1000.0f));
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
