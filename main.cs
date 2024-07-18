using ImGuiNET;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;

using NVector2 = System.Numerics.Vector2;
using NVector3 = System.Numerics.Vector3;
using NVector4 = System.Numerics.Vector4;

namespace PathFind3D
{
    public enum DrawMode { Wireframe, Wall, Air, Start, End, Open, Closed, Path, PathAstar, PathBFS }
    public class main
    {
        public main(int resX, int resY, string title)
        {
            this.resX = resX;
            this.resY = resY;
            this.title = title;
        }

        #region global variables

        ImGuiController _controller;
        private int resX;
        private int resY;
        private string title;
        private Action? runAction = null;
        private GameWindow? window = null;
        private float aspectRatio = 16f / 9;
        private float nodeDistance = 0.125f;
        private Random rng = new(DateTime.Now.Millisecond * DateTime.Now.Second);
        private static Vector3i gridSize = new(32);
        private bool exit = false;
        private Matrix4 projectionMatrix;
        private Matrix4 viewMatrix;
        private static float zoom = gridSize.X / 4;
        private Vector3 cameraPosition = new Vector3(0, 0, zoom);
        private GraphNode[,,] grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];
        private Vector3 rot = new(0);

        private double nodeDensity = 0;

        private bool continueSearch = true;
        private List<GraphNode> openSet = new();
        private SortedSet<GraphNode> openSetSorted = new();
        private List<GraphNode> closedSet = new();
        private GraphNode? boxNode;
        private float baseSize;
        private bool drawWalls = true;
        private Vector3i startnodePos = (0, 0, 0);
        private Vector3i endNodePos = (gridSize.X, gridSize.Y, gridSize.Z);


        List<GraphNode> AstarPath = new();
        List<GraphNode> BFSPath = new();

        //fps calculation stuff
        private double elapsedTime = 0.0;
        private int frameCount = 0;


        Thread? BFSThread;
        Thread? AStarThread;


        Vector3i[] mainDirections =
        {
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1)
        };
        #endregion

        #region matrices

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

        #endregion

        #region grid and config

        private void reloadConfig()
        {
            GridConfig gconf = new();
            gconf.ReadConfigFromFile("gridSettings.txt");
            gridSize = gconf.GridSize;
            nodeDensity = gconf.NodeDensity;
            startnodePos = MtH.Clamp(gconf.StartNodePos, (0, 0, 0), gridSize);
            endNodePos = MtH.Clamp(gconf.EndNodePos, (0, 0, 0), gridSize);
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

        private void runRebuildGridThread(ref GraphNode[,,] grid)
        {
            if (grid.GetUpperBound(0) != gridSize.X || grid.GetUpperBound(1) != gridSize.Y || grid.GetUpperBound(2) != gridSize.Z)
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
                            nd.DrawMD = DrawMode.Air;

                        else
                            nd.DrawMD = DrawMode.Wall;

                        if (i == startnodePos.X && j == startnodePos.Y && k == startnodePos.Z)
                            nd.DrawMD = DrawMode.Start;

                        if (i == endNodePos.X - 1 && j == endNodePos.Y - 1 && k == endNodePos.Z - 1)
                            nd.DrawMD = DrawMode.End;

                        nd.dstToEnd = (nd.GridPosition - endNodePos).EuclideanLength;

                        grid[i, j, k] = nd;
                    }
                }
            }
        }

        private void drawGrid()
        {
            foreach (GraphNode? node in grid)
            {
                if (node?.DrawMD != DrawMode.Air)

                    if (node?.DrawMD == DrawMode.Wall && drawWalls)
                        node?.Draw(viewMatrix, projectionMatrix);

                    else if (node?.DrawMD != DrawMode.Wall)
                        node?.Draw(viewMatrix, projectionMatrix);
            }
        }

        private void updateGrid()
        {
            foreach (GraphNode node in grid)
            {
                if (node != null && node.DrawMD != DrawMode.Air)
                {
                    if (node.Rotation != rot)
                        node.Rotation = rot;

                    if (node.AspectRatio != aspectRatio)
                        node.AspectRatio = aspectRatio;
                }
            }
        }

        #endregion

        #region pathfinding

        #region BFS
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

                if (neighbor.dstFromStart >= currentNode.dstFromStart + 1 || neighbor.dstFromStart == 0)
                {
                    neighbor.Parent = currentNode;
                    neighbor.dstFromStart = currentNode.dstFromStart + 1;
                }

                if (neighbor.DrawMD == DrawMode.End)
                {
                    neighbor.Parent = currentNode;
                    continueSearch = false;
                    openSet.Clear();
                    closedSet.Clear();
                    Console.WriteLine("PathFound");
                    grid[startnodePos.X, startnodePos.Y, startnodePos.Z].Parent = null;
                    BacktrackPath(neighbor);

                    foreach (var item in grid)
                    {
                        if (item.DrawMD == DrawMode.Open)
                        {
                            item.DrawMD = DrawMode.Air;
                        }
                    }

                    //// Cache the path
                    //GraphNode cacheNode = neighbor;
                    //BFSPath.Clear(); // Clear any previous path
                    //while (cacheNode.Parent != null)
                    //{
                    //    BFSPath.Add(cacheNode);
                    //    cacheNode = cacheNode.Parent;
                    //}

                    return;
                }

                if (neighbor.DrawMD == DrawMode.Air)
                {
                    openSet.Add(neighbor);
                }
            }
        }
        private void BreadthFirstSearch(GraphNode startNode)
        {
            //if (BFSPath.Count > 0)
            //{
            //    // Use the cached path if it exists
            //    foreach (var node in BFSPath)
            //    {
            //        node.DrawMD = DrawMode.Path;
            //    }
            //    Console.WriteLine("Using cached path.");
            //    return;
            //}

            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            openSet.Clear();
            closedSet.Clear();
            openSet.Add(startNode);

            while (openSet.Count > 0 && continueSearch)
            {
                GraphNode currentNode = openSet[0];
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

            stopwatch.Stop();
            TimeSpan elapsedTime = stopwatch.Elapsed;
            Console.WriteLine($"BreadthFirstSearch execution time: {elapsedTime.TotalMilliseconds} ms");
        }
        #endregion

        #region A*
        private void AddNeighborsToSortedOpenSet(GraphNode currentNode)
        {
            lock (grid)
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

                    lock (neighbor)
                    {
                        if (neighbor.dstFromStart >= currentNode.dstFromStart + 1 || neighbor.dstFromStart == 0)
                        {
                            neighbor.Parent = currentNode;
                            neighbor.dstFromStart = currentNode.dstFromStart + 1;
                        }

                        if (closedSet.Contains(neighbor) || openSetSorted.Contains(neighbor))
                        {
                            continue;
                        }

                        if (neighbor.DrawMD == DrawMode.End)
                        {
                            neighbor.Parent = currentNode;
                            continueSearch = false;
                            openSetSorted.Clear();
                            closedSet.Clear();
                            Console.WriteLine("PathFound");
                            BacktrackPath(neighbor);
                            foreach (var item in grid)
                            {
                                if (item.DrawMD == DrawMode.Open)
                                {
                                    item.DrawMD = DrawMode.Air;
                                }
                            }
                            return;
                        }

                        if (neighbor.DrawMD == DrawMode.Air)
                        {
                            openSetSorted.Add(neighbor);
                        }
                    }
                }
            }
        }
        public void AStar(GraphNode startNode)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            openSetSorted = new(Comparer<GraphNode>.Create((x, y) => (x.dstToEnd - x.dstFromStart).CompareTo(y.dstToEnd - y.dstFromStart)));
            openSetSorted.Clear();
            closedSet.Clear();

            openSetSorted.Add(startNode);

            while (openSetSorted.Count > 0 && continueSearch)
            {
                GraphNode currentNode = openSetSorted.First();
                openSetSorted.Remove(currentNode);
                closedSet.Add(currentNode);

                lock (grid)
                {
                    foreach (GraphNode node in openSetSorted)
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

                    AddNeighborsToSortedOpenSet(currentNode);
                }
            }

            openSetSorted.Clear();
            closedSet.Clear();

            stopwatch.Stop();
            TimeSpan elapsedTime = stopwatch.Elapsed;
            Console.WriteLine($"A* execution time: {elapsedTime.TotalMilliseconds} ms");
        }
        #endregion

        private void BacktrackPath(GraphNode endNode)
        {
            int PathLen = 0;
            GraphNode currentNode = endNode;
            while (currentNode.Parent != null && currentNode.DrawMD != DrawMode.Start)
            {
                currentNode = currentNode.Parent;
                if (currentNode.DrawMD != DrawMode.Start)
                {
                    currentNode.DrawMD = DrawMode.Path;
                    PathLen++;
                }
            }
            Console.WriteLine($"Path length: {endNode.dstFromStart - 1}");
            updateGrid();
        }

        #endregion

        #region GUI

        bool _showPopup = false;
        bool _isMouseOverMenu = false;
        private void ProcessGUI()
        {
            ImGui.Begin("Config");
            ImGui.SetWindowFontScale(1.2f);

            NVector2 windowPos = ImGui.GetWindowPos();
            NVector2 windowSize = ImGui.GetWindowSize();
            NVector2 mousePos = ImGui.GetMousePos();

            _isMouseOverMenu = mousePos.X >= windowPos.X &&
                               mousePos.X <= windowPos.X + windowSize.X &&
                               mousePos.Y >= windowPos.Y &&
                               mousePos.Y <= windowPos.Y + windowSize.Y;


            if (ImGui.Button("Rebuild Grid"))
            {
                reloadConfig();
                rebuildGrid();
            }

            if (ImGui.Button(drawWalls == true ? "Hide Walls" : "UnHide Walls"))
            {
                drawWalls = !drawWalls;
            }

            if (ImGui.Button("Clear Path"))
            {

            }

            if (ImGui.Button("run BFS"))
            {
                grid[startnodePos.X, startnodePos.Y, startnodePos.Z].DrawMD = DrawMode.Start;
                grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1].DrawMD = DrawMode.End;
                BFSThread = new Thread(() =>
                {
                    BreadthFirstSearch(grid[startnodePos.X, startnodePos.Y, startnodePos.Z]);
                });

                BFSThread.Start();
            }

            if (ImGui.Button("run A*"))
            {
                grid[startnodePos.X, startnodePos.Y, startnodePos.Z].DrawMD = DrawMode.Start;
                grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1].DrawMD = DrawMode.End;
                AStarThread = new Thread(() =>
                {
                    AStar(grid[startnodePos.X, startnodePos.Y, startnodePos.Z]);
                });

                AStarThread.Start();
            }

            //ImGui.SetWindowFontScale(originalScale);
            ImGui.End();
        }

        #endregion
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
                reloadConfig();

                BFSPath.Clear();
                rebuildGrid();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.F))
            {
                if (BFSPath.Count > 0)
                {
                    foreach (GraphNode pathNode in BFSPath)
                    {
                        pathNode.Draw(viewMatrix, projectionMatrix);
                    }
                }
                else
                {
                    grid[startnodePos.X, startnodePos.Y, startnodePos.Z].DrawMD = DrawMode.Start;
                    grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1].DrawMD = DrawMode.End;
                    BFSThread = new Thread(() =>
                    {
                        BreadthFirstSearch(grid[startnodePos.X, startnodePos.Y, startnodePos.Z]);
                    });

                    BFSThread.Start();
                }
            }

            if (window.KeyboardState.IsKeyPressed(Keys.G))
            {
                grid[startnodePos.X, startnodePos.Y, startnodePos.Z].DrawMD = DrawMode.Start;
                grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1].DrawMD = DrawMode.End;
                AStarThread = new Thread(() =>
                {
                    AStar(grid[startnodePos.X, startnodePos.Y, startnodePos.Z]);
                });

                AStarThread.Start();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.Q))
            {
                continueSearch = false;

                AStarThread?.Join();
                BFSThread?.Join();
                openSetSorted.Clear();
                openSet.Clear();
                closedSet.Clear();

                foreach (GraphNode node in grid)
                {
                    if (node.DrawMD == DrawMode.Closed || node.DrawMD == DrawMode.Open || node.DrawMD == DrawMode.Path)
                        node.DrawMD = DrawMode.Air;

                    node.Parent = null;
                    node.dstFromStart = 0;
                }

                continueSearch = true;
            }

            if (window.KeyboardState.IsKeyPressed(Keys.H))
            {
                drawWalls = !drawWalls;
            }
        }

        private void calculateFPS(double time)
        {
            if (window == null) return;
            elapsedTime += time;
            frameCount++;
            if (elapsedTime >= 1.0)
            {
                double fps = frameCount / elapsedTime;
                window.Title = $"{title} - FPS: {fps:F2} - Grid Dimensions: {gridSize}";
                elapsedTime = 0.0;
                frameCount = 0;
            }
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

        public bool Initialize()
        {
            window = new GameWindow(
                GameWindowSettings.Default,
                new NativeWindowSettings
                {
                    ClientSize = (resX, resY),
                    Title = title,
                    Profile = ContextProfile.Compatability,
                    API = ContextAPI.OpenGL,
                    Flags = ContextFlags.Default,
                    Vsync = VSyncMode.Adaptive
                }
            );

            reloadConfig();

            rebuildGrid();

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

            baseSize = MtH.cubeRootF(gridSize.X * gridSize.Y * gridSize.Z) * 0.125f + 0.01f;

            _controller = new ImGuiController(window.ClientSize.X, window.ClientSize.Y);

            boxNode = new GraphNode(0.125f / 2, 0.125f / 2, 0.125f / 2)
            {
                BaseSize = baseSize,
                DrawMD = DrawMode.Wireframe
            };
            return true;
        }

        private void onUpdateFrame(FrameEventArgs e)
        {
            if (window == null)
                return;

            updateGrid();

            if ((boxNode?.Rotation != rot || boxNode?.AspectRatio != aspectRatio) && boxNode != null)
            {
                boxNode.Rotation = rot;
                boxNode.AspectRatio = aspectRatio;
            }

            calculateFPS(e.Time);
            handleUserInput();

            runAction?.Invoke();
        }

        private void onRenderFrame(FrameEventArgs args)
        {

            if (window == null || grid == null)
                return;

            _controller.Update(window, (float)args.Time);

            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit | ClearBufferMask.StencilBufferBit);
            //ImGui.DockSpaceOverViewport();  // doesen`t work for some reason

            drawGrid();

            ProcessGUI();

            boxNode?.Draw(viewMatrix, projectionMatrix);

            _controller.Render();
            ImGuiController.CheckGLError("End of frame");
            window.SwapBuffers();
        }

        private void onResize(ResizeEventArgs args)
        {
            if (window == null)
                return;

            aspectRatio = (float)args.Width / args.Height;
            GL.Viewport(0, 0, args.Width, args.Height);
            _controller.WindowResized(args.Width, args.Height);

            float fov = MathHelper.DegreesToRadians(45.0f);
            projectionMatrix = Matrix4.CreatePerspectiveFieldOfView(fov, aspectRatio, 0.1f, 100.0f);

            // Inform ImGui of the resize
            ImGui.GetIO().DisplaySize = new NVector2(args.Width, args.Height);
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

            if (_isMouseOverMenu)
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
