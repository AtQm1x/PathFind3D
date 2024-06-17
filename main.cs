using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTKBase;
using PathFind3D;
using System;
using System.Collections.Generic;
using System.Linq;
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
        private static Vector3i gridSize = new(32);
        private bool exit = false;
        private Matrix4 projectionMatrix;
        private Matrix4 viewMatrix;
        private float zoom = 1.0f;
        private Vector3 cameraPosition = new Vector3(0, 0, 1);
        private GraphNode[,,] grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];
        private Vector3 rot = new(0);
        private double nodeDensity = 0.25;

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
            GL.BlendFunc(BlendingFactor.One, BlendingFactor.OneMinusDstAlpha);

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
            continueSearch = false;
        }

        public void Run(Action mainLoopFunction)
        {
            runAction = mainLoopFunction;

            window?.Run();
        }

        private void rebuildGrid()
        {
            continueSearch = false;
            Thread thread = new Thread(() =>
            {
                runRebuildGridThread(ref grid);
            });

            thread.Start();
            continueSearch = true;
        }

        Vector3i startPos = (0, 0, 0);
        Vector3i endPos = (gridSize.X, gridSize.Y, gridSize.Z);

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
                            AspectRatio = aspectRatio,
                            GridPosition = (i, j, k)
                        };
                        if (rng.NextDouble() >= nodeDensity)
                            nd.DrawMD = DrawMode.None;

                        else
                            nd.DrawMD = DrawMode.Both;

                        if (i == startPos.X && j == startPos.Y && k == startPos.Z)
                            nd.DrawMD = DrawMode.Start;

                        if (i == endPos.X - 1 && j == endPos.Y - 1 && k == endPos.Z - 1)
                            nd.DrawMD = DrawMode.End;

                        grid[i, j, k] = nd;
                    }
                }
            }
        }

        private void handleUserInput()
        {
            if (window == null)
                return;
            if (window.KeyboardState.IsKeyDown(Keys.Escape))
            {
                Shutdown();
                window.Close();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.R))
            {
                rebuildGrid();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.F))
            {
                Thread thread = new Thread(() =>
                {
                    BreadthFirstSearch(grid[0, 0, 0]);
                });

                thread.Start();
            }
        }

        Vector3i[] mainDirections =
        {
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1)
        };

        private void AddNeighborsToOpenSet(GraphNode currentNode)
        {
            Vector3i nodePos = currentNode.GridPosition;
            foreach (var direction in mainDirections)
            {
                Vector3i newNodePos = nodePos + direction;

                if (newNodePos.X < 0 || newNodePos.Y < 0 || newNodePos.Z < 0 ||
                    newNodePos.X >= gridSize.X || newNodePos.Y >= gridSize.Y || newNodePos.Z >= gridSize.Z)
                {
                    continue;
                }

                GraphNode neighbor = grid[newNodePos.X, newNodePos.Y, newNodePos.Z];

                if (closedSet.Contains(neighbor) || openSet.Contains(neighbor))
                {
                    continue;
                }

                if (neighbor.DrawMD == DrawMode.End)
                {
                    neighbor.Parent = currentNode;
                    continueSearch = false;
                    openSet.Clear();
                    closedSet.Clear();
                    Console.WriteLine("PathFound");
                    BacktrackPath(neighbor);
                    foreach (var item in grid)
                    {
                        if (item.DrawMD == DrawMode.Open)
                        {
                            item.DrawMD = DrawMode.None;
                        }

                        if (item.DrawMD == DrawMode.Both)
                        {
                            item.DrawMD = DrawMode.Glass;
                        }
                    }
                    return;
                }

                if (neighbor.DrawMD == DrawMode.None)
                {
                    neighbor.Parent = currentNode;
                    openSet.Add(neighbor);
                }
            }
        }

        bool continueSearch = true;
        List<GraphNode> openSet = new();
        List<GraphNode> closedSet = new();

        private void BreadthFirstSearch(GraphNode startNode)
        {
            openSet.Clear();
            closedSet.Clear();

            openSet.Add(startNode);

            while (openSet.Count > 0 && continueSearch)
            {
                GraphNode currentNode = openSet.First();
                openSet.Remove(currentNode);
                closedSet.Add(currentNode);

                foreach (GraphNode node in openSet)
                {
                    if (node.DrawMD != DrawMode.Open)
                    {
                        node.DrawMD = DrawMode.Open;
                    }
                }

                foreach (GraphNode node in closedSet)
                {
                    if (node.DrawMD != DrawMode.Closed && node.DrawMD != DrawMode.Start && node.DrawMD != DrawMode.End)
                    {
                        node.DrawMD = DrawMode.Closed;
                    }
                }

                AddNeighborsToOpenSet(currentNode);
            }
        }

        private void BacktrackPath(GraphNode endNode)
        {
            GraphNode currentNode = endNode;
            while (currentNode.Parent != null)
            {
                currentNode = currentNode.Parent;
                if (currentNode.DrawMD != DrawMode.Start)
                    currentNode.DrawMD = DrawMode.Path;
            }
            Console.WriteLine("Path has been traced back.");
        }
        private void onUpdateFrame(FrameEventArgs e)
        {
            if (window == null)
                return;

            foreach (GraphNode node in grid)
            {
                if (node != null && (node.DrawMD != DrawMode.None || node.DrawMD == DrawMode.Closed))
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

            handleUserInput();

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
