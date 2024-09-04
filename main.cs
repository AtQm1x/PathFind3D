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
using System.Threading.Tasks;
using NVector2 = System.Numerics.Vector2;
using NVector3 = System.Numerics.Vector3;
using NVector4 = System.Numerics.Vector4;

namespace PathFind3D
{
    // enum for different rendering modes
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
        private static Vector3i gridSize = new(32, 16, 32);
        private bool exit = false;
        private Matrix4 projectionMatrix;
        private Matrix4 viewMatrix;
        private static float zoom = gridSize.X / 4;
        private Vector3 cameraPosition = new Vector3(0, 0, zoom);
        private GraphNode[,,] grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];
        private Vector3 rot = new(0);

        private double nodeDensity = 0.1;

        private bool continueSearch = true;
        private List<GraphNode> openSet = new();
        private SortedSet<GraphNode> openSetSorted = new();
        private List<GraphNode> closedSet = new();
        private GraphNode? boxNode;
        private Vector3 baseSize;
        private bool drawWalls = true;
        private Vector3i startNodePos = (0, 0, 0);
        private Vector3i endNodePos = (gridSize.X, gridSize.Y, gridSize.Z);


        List<GraphNode> AstarPath = new();
        List<GraphNode> BFSPath = new();

        // fps calculation variables
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
            // setup projection matrix
            float fov = MathHelper.DegreesToRadians(45.0f);
            projectionMatrix = Matrix4.CreatePerspectiveFieldOfView(fov, aspectRatio, 0.1f, 100.0f);
            UpdateViewMatrix();
        }
        private void UpdateViewMatrix()
        {
            // update view matrix based on camera position
            viewMatrix = Matrix4.LookAt(
                cameraPosition,
                (cameraPosition.X, cameraPosition.Y, cameraPosition.Z - 1),
                Vector3.UnitY
            );
        }

        #endregion

        #region grid and config

        private void rebuildGrid()
        {
            // stop any ongoing search
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
            // create a box node to represent the entire grid
            boxNode = new GraphNode(Vector3i.Zero)
            {
                BaseSize = gridSize * new Vector3(0.125f) + new Vector3(0.01f),
                DrawMD = DrawMode.Wireframe
            };

            // recreate grid if size has changed
            if (grid.LongLength != gridSize.X * gridSize.Y * gridSize.Z)
                grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];

            Vector3 startOffset = (gridSize - Vector3.One) * new Vector3(nodeDistance / 2);

            GraphNode[,,] gridBuffer = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];

            // populate grid with nodes
            for (int i = 0; i < gridSize.X; i++)
            {
                for (int j = 0; j < gridSize.Y; j++)
                {
                    for (int k = 0; k < gridSize.Z; k++)
                    {
                        float posX = i * nodeDistance - startOffset.X;
                        float posY = j * nodeDistance - startOffset.Y;
                        float posZ = k * nodeDistance - startOffset.Z;
                        GraphNode nd = new(posX, posY, posZ)
                        {
                            Rotation = rot,
                            AspectRatio = aspectRatio,
                            GridPosition = (i, j, k)
                        };

                        // randomly set node as air or wall
                        if (rng.NextDouble() >= nodeDensity)
                            nd.DrawMD = DrawMode.Air;
                        else
                            nd.DrawMD = DrawMode.Wall;

                        // set start and end nodes
                        if (i == startNodePos.X && j == startNodePos.Y && k == startNodePos.Z)
                            nd.DrawMD = DrawMode.Start;

                        if (i == endNodePos.X - 1 && j == endNodePos.Y - 1 && k == endNodePos.Z - 1)
                            nd.DrawMD = DrawMode.End;

                        gridBuffer[i, j, k] = nd;
                    };
                };
            };

            grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];
            grid = gridBuffer;
        }

        private void drawGrid()
        {
            // sort transparent objects for correct rendering
            var sortedGrid = grid.Cast<GraphNode>()
                .Where(node => node?.DrawMD != DrawMode.Air && node?.DrawMD != DrawMode.Wall && node != null)
                .OrderByDescending(node => Vector3.Distance(cameraPosition, new Vector3(node.Position.X, node.Position.X, node.Position.X)))
                .ToList();

            // draw opaque objects (walls)
            foreach (var node in grid)
            {
                if (node != null)
                {
                    if (node.DrawMD == DrawMode.Wall && drawWalls)
                        node.Draw(viewMatrix, projectionMatrix);
                }
            }

            // draw transparent objects
            foreach (var node in sortedGrid)
            {
                if (node != null)
                {
                    node?.Draw(viewMatrix, projectionMatrix);
                }
            }
        }

        private void updateGrid()
        {
            // update rotation and aspect ratio for all visible nodes
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

        Vector3i[] directions = new Vector3i[]
        {
            new Vector3i(1, 1, 1), new Vector3i(1, 1, 0), new Vector3i(1, 1, -1),
            new Vector3i(1, 0, 1), new Vector3i(1, 0, 0), new Vector3i(1, 0, -1),
            new Vector3i(1, -1, 1), new Vector3i(1, -1, 0), new Vector3i(1, -1, -1),
            new Vector3i(0, 1, 1), new Vector3i(0, 1, 0), new Vector3i(0, 1, -1),
            new Vector3i(0, 0, 1), new Vector3i(0, 0, -1),
            new Vector3i(0, -1, 1), new Vector3i(0, -1, 0), new Vector3i(0, -1, -1),
            new Vector3i(-1, 1, 1), new Vector3i(-1, 1, 0), new Vector3i(-1, 1, -1),
            new Vector3i(-1, 0, 1), new Vector3i(-1, 0, 0), new Vector3i(-1, 0, -1),
            new Vector3i(-1, -1, 1), new Vector3i(-1, -1, 0), new Vector3i(-1, -1, -1),
        };

        #region BFS
        private void AddNeighborsToOpenSet(GraphNode currentNode)
        {
            Vector3i nodePos = currentNode.GridPosition;
            foreach (var direction in mainDirections)
            {
                Vector3i newNodePos = nodePos + direction;

                // check if new position is within grid bounds
                if (newNodePos.X < 0 || newNodePos.Y < 0 || newNodePos.Z < 0 ||
                    newNodePos.X >= gridSize.X || newNodePos.Y >= gridSize.Y || newNodePos.Z >= gridSize.Z)
                {
                    continue;
                }

                GraphNode neighbor = grid[newNodePos.X, newNodePos.Y, newNodePos.Z];

                // skip if neighbor is already processed or in queue
                if (closedSet.Contains(neighbor) || openSet.Contains(neighbor))
                {
                    continue;
                }

                // update distance from start if shorter path found
                if (neighbor.dstFromStart >= currentNode.dstFromStart + 1 || neighbor.dstFromStart == 0)
                {
                    neighbor.Parent = currentNode;
                    neighbor.dstFromStart = currentNode.dstFromStart + 1;
                }

                // check if end node reached
                if (neighbor.DrawMD == DrawMode.End)
                {
                    neighbor.Parent = currentNode;
                    continueSearch = false;
                    openSet.Clear();
                    closedSet.Clear();
                    Console.WriteLine("PathFound");
                    grid[startNodePos.X, startNodePos.Y, startNodePos.Z].Parent = null;
                    BacktrackPath(neighbor);

                    // reset open nodes to air
                    foreach (var item in grid)
                    {
                        if (item.DrawMD == DrawMode.Open)
                        {
                            item.DrawMD = DrawMode.Air;
                        }
                    }

                    return;
                }

                // add air nodes to open set
                if (neighbor.DrawMD == DrawMode.Air)
                {
                    openSet.Add(neighbor);
                }
            }
        }
        private void BreadthFirstSearch(GraphNode startNode)
        {
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

                // update node colors for visualization
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
        private void AddNeighborsToOpenSortedSet(GraphNode currentNode, Vector3i endNodePos, PriorityQueue<GraphNode, float> openSet, HashSet<GraphNode> closedSet)
        {
            foreach (Vector3i direction in directions)
            {
                Vector3i newNodePos = currentNode.GridPosition + direction;

                // check if new position is within grid bounds
                if (newNodePos.X < 0 || newNodePos.Y < 0 || newNodePos.Z < 0 ||
                    newNodePos.X >= gridSize.X || newNodePos.Y >= gridSize.Y || newNodePos.Z >= gridSize.Z)
                {
                    continue;
                }

                GraphNode neighbor = grid[newNodePos.X, newNodePos.Y, newNodePos.Z];

                // skip if neighbor is in closed set or is a wall
                if (closedSet.Contains(neighbor) || neighbor.DrawMD == DrawMode.Wall)
                {
                    continue;
                }

                float tentativeGScore = currentNode.gScore + direction.EuclideanLength;

                // update node if better path found
                if (!openSet.UnorderedItems.Any(x => x.Element == neighbor) || tentativeGScore < neighbor.gScore)
                {
                    neighbor.Parent = currentNode;
                    neighbor.gScore = tentativeGScore;
                    neighbor.hScore = (neighbor.GridPosition - endNodePos).EuclideanLengthSquared;
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;

                    if (!openSet.UnorderedItems.Any(x => x.Element == neighbor))
                    {
                        openSet.Enqueue(neighbor, neighbor.fScore);
                        if (neighbor.DrawMD != DrawMode.Start && neighbor.DrawMD != DrawMode.End)
                        {
                            neighbor.DrawMD = DrawMode.Open;
                        }
                    }
                    else
                    {
                        openSet.EnqueueDequeue(neighbor, neighbor.fScore);
                    }
                }
            }
        }
        public void AStar(Vector3i startNodePos, Vector3i endNodePos)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            PriorityQueue<GraphNode, float> openSet = new PriorityQueue<GraphNode, float>();
            HashSet<GraphNode> closedSet = new HashSet<GraphNode>();

            GraphNode startNode = grid[this.startNodePos.X, this.startNodePos.Y, this.startNodePos.Z];

            // initialize start node
            startNode.gScore = 0;
            startNode.hScore = (startNodePos - endNodePos).EuclideanLengthSquared;
            startNode.fScore = startNode.hScore;
            openSet.Enqueue(startNode, startNode.fScore);

            while (openSet.Count > 0)
            {
                GraphNode currentNode = openSet.Dequeue();

                // check if we've reached the end node
                if (currentNode.DrawMD == DrawMode.End)
                {
                    Console.WriteLine("Path found!");
                    BacktrackPath(currentNode);
                    break;
                }

                closedSet.Add(currentNode);

                lock (grid)
                {
                    // update node color for visualization
                    if (currentNode.DrawMD != DrawMode.Start && currentNode.DrawMD != DrawMode.End)
                    {
                        currentNode.DrawMD = DrawMode.Closed;
                    }

                    AddNeighborsToOpenSortedSet(currentNode, endNodePos, openSet, closedSet);
                }
            }

            // no path found
            if (openSet.Count == 0)
            {
                Console.WriteLine("No path found");
            }

            stopwatch.Stop();
            TimeSpan elapsedTime = stopwatch.Elapsed;
            Console.WriteLine($"A* execution time: {elapsedTime.TotalMilliseconds} ms");
        }
        #endregion

        private void BacktrackPath(GraphNode endNode)
        {
            int PathLen = 0;
            GraphNode currentNode = endNode;

            // trace path from end to start
            while (currentNode.Parent != null && currentNode.DrawMD != DrawMode.Start)
            {
                currentNode = currentNode.Parent;
                if (currentNode.DrawMD != DrawMode.Start)
                {
                    currentNode.DrawMD = DrawMode.Path;
                    PathLen++;
                }
            }
            foreach (var item in grid)
            {
                if (item.DrawMD == DrawMode.Closed || item.DrawMD == DrawMode.Open)
                {
                    item.DrawMD = DrawMode.Air;
                }
            }
            Console.WriteLine($"Path length: {endNode.dstFromStart - 1}");
            updateGrid();
        }

        #endregion

        #region GUI

        bool _input_grid_size = false;
        bool _showPopup = false;
        bool _isMouseOverMenu = false;

        int gsA = gridSize.X, gsB = gridSize.Y, gsC = gridSize.Z;
        private void ProcessGUI()
        {
            ImGui.DockSpaceOverViewport(ImGui.GetMainViewport().ID, ImGui.GetMainViewport(), ImGuiDockNodeFlags.PassthruCentralNode);

            ImGui.Begin("Config");
            ImGui.SetWindowFontScale(1.2f);

            // check if mouse is over the ImGui window
            NVector2 windowPos = ImGui.GetWindowPos();
            NVector2 windowSize = ImGui.GetWindowSize();
            NVector2 mousePos = ImGui.GetMousePos();

            _isMouseOverMenu = (mousePos.X >= windowPos.X &&
                               mousePos.X <= windowPos.X + windowSize.X &&
                               mousePos.Y >= windowPos.Y &&
                               mousePos.Y <= windowPos.Y + windowSize.Y);

            // gui buttons for various actions
            if (ImGui.Button("Rebuild Grid"))
            {
                rebuildGrid();
            }

            if (ImGui.Button(drawWalls == true ? "Hide Walls" : "UnHide Walls"))
            {
                drawWalls = !drawWalls;
            }

            if (ImGui.Button("Clear Path"))
            {
                clearPath();
            }

            if (ImGui.Button("run BFS"))
            {
                grid[startNodePos.X, startNodePos.Y, startNodePos.Z].DrawMD = DrawMode.Start;
                grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1].DrawMD = DrawMode.End;
                BFSThread = new Thread(() =>
                {
                    BreadthFirstSearch(grid[startNodePos.X, startNodePos.Y, startNodePos.Z]);
                });

                BFSThread.Start();
            }

            if (ImGui.Button("run A*"))
            {
                grid[startNodePos.X, startNodePos.Y, startNodePos.Z].DrawMD = DrawMode.Start;
                grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1].DrawMD = DrawMode.End;
                AStarThread = new Thread(() =>
                {
                    AStar(startNodePos, endNodePos);
                });

                AStarThread.Start();
            }

            if (ImGui.Button(_input_grid_size == false ? "Open Grid Config" : "Close Grid Config"))
            {
                _input_grid_size = !_input_grid_size;
            }

            if (_input_grid_size)
            {
                ImGui.Begin("Grid Size Input");
                // check if mouse is over the ImGui window
                windowPos = ImGui.GetWindowPos();
                windowSize = ImGui.GetWindowSize();
                mousePos = ImGui.GetMousePos();

                _isMouseOverMenu = _isMouseOverMenu || (mousePos.X >= windowPos.X &&
                                   mousePos.X <= windowPos.X + windowSize.X &&
                                   mousePos.Y >= windowPos.Y &&
                                   mousePos.Y <= windowPos.Y + windowSize.Y);



                ImGui.InputInt("X", ref gsA);
                ImGui.InputInt("Y", ref gsB);
                ImGui.InputInt("Z", ref gsC);

                if (ImGui.Button("Apply"))
                {
                    gsA = Math.Max(gsA, 1);
                    gsB = Math.Max(gsB, 1);
                    gsC = Math.Max(gsC, 1);
                    gridSize = (gsA, gsB, gsC);
                    startNodePos = Vector3i.Clamp(startNodePos, Vector3i.Zero, gridSize);
                    endNodePos = Vector3i.Clamp(endNodePos, Vector3i.Zero, gridSize);
                    rebuildGrid();
                }
            }

            ImGui.End();
        }

        #endregion

        private void handleUserInput()
        {
            if (window == null)
                return;

            // handle keyboard input
            if (window.KeyboardState.IsKeyDown(Keys.Escape))
            {
                Shutdown();
                window.Close();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.R))
            {
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
                    grid[startNodePos.X, startNodePos.Y, startNodePos.Z].DrawMD = DrawMode.Start;
                    grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1].DrawMD = DrawMode.End;
                    BFSThread = new Thread(() =>
                    {
                        BreadthFirstSearch(grid[startNodePos.X, startNodePos.Y, startNodePos.Z]);
                    });

                    BFSThread.Start();
                }
            }

            if (window.KeyboardState.IsKeyPressed(Keys.G))
            {
                grid[startNodePos.X, startNodePos.Y, startNodePos.Z].DrawMD = DrawMode.Start;
                grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1].DrawMD = DrawMode.End;
                AStarThread = new Thread(() =>
                {
                    AStar(startNodePos, endNodePos);
                });

                AStarThread.Start();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.Q))
            {
                clearPath();
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
        private void clearPath()
        {

            // reset search state
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
            // create and configure the game window
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

            rebuildGrid();

            // set up event handlers
            window.RenderFrame += onRenderFrame;
            window.UpdateFrame += onUpdateFrame;
            window.Resize += onResize;
            window.MouseWheel += onMouseWheel;
            window.MouseMove += onMouseMove;
            window.TextInput += onTextInput;

            GL.Viewport(0, 0, resX, resY);

            // enable depth testing
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Less);

            // enable alpha blending
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactor.SrcAlpha, BlendingFactor.OneMinusSrcAlpha);
            GL.BlendEquation(BlendEquationMode.FuncAdd);

            // enable face culling
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.FrontFace(FrontFaceDirection.Ccw);

            SetupMatrices();

            baseSize = new Vector3(gridSize.X, gridSize.Y, gridSize.Z) * 0.125f + (0.01f, 0.01f, 0.01f);

            _controller = new ImGuiController(window.ClientSize.X, window.ClientSize.Y);

            var io = ImGui.GetIO();
            io.ConfigFlags |= ImGuiConfigFlags.DockingEnable;
            io.ConfigFlags |= ImGuiConfigFlags.ViewportsEnable;
            io.ConfigViewportsNoAutoMerge = true;
            io.ConfigViewportsNoTaskBarIcon = true;

            boxNode = new GraphNode(Vector3i.Zero)
            {
                BaseSize = gridSize * new Vector3(0.125f) + new Vector3(0.01f),
                DrawMD = DrawMode.Wireframe
            };
            return true;
        }

        private void onTextInput(TextInputEventArgs e)
        {
            _controller.PressChar((char)e.Unicode);
        }

        private void onUpdateFrame(FrameEventArgs e)
        {
            if (window == null)
                return;

            updateGrid();

            // update box if necessary
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

            GL.ClearColor(0.2f, 0.3f, 0.4f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            GL.Viewport(0, 0, window.ClientSize.X, window.ClientSize.Y);

            GL.Enable(EnableCap.DepthTest);

            drawGrid();
            boxNode?.Draw(viewMatrix, projectionMatrix);

            GL.Disable(EnableCap.DepthTest);

            _controller.Update(window, (float)args.Time);
            ProcessGUI();
            _controller.Render();

            ImGuiController.CheckGLError("End of frame");
            window.SwapBuffers();
        }

        private void onResize(ResizeEventArgs args)
        {
            if (window == null)
                return;

            // update aspect ratio and viewport
            aspectRatio = (float)args.Width / args.Height;
            GL.Viewport(0, 0, args.Width, args.Height);
            _controller.WindowResized(args.Width, args.Height);

            // update projection matrix
            float fov = MathHelper.DegreesToRadians(45.0f);
            projectionMatrix = Matrix4.CreatePerspectiveFieldOfView(fov, aspectRatio, 0.1f, 100.0f);

            // inform ImGui of the resize
            ImGui.GetIO().DisplaySize = new NVector2(args.Width, args.Height);
        }

        private void onMouseWheel(MouseWheelEventArgs e)
        {
            // zoom
            zoom -= e.OffsetY * 0.2f;
            zoom = Math.Max(0.1f, Math.Min(zoom, 1000.0f));
            cameraPosition.Z = zoom;
            UpdateViewMatrix();
            _controller.MouseScroll(e.Offset);
        }

        private void onMouseMove(MouseMoveEventArgs e)
        {
            if (window == null)
                return;

            if (_isMouseOverMenu)
                return;

            // camera rotation
            if (window.MouseState.IsButtonDown(MouseButton.Left))
            {
                rot.Y += (window.MouseState.Delta.X / 100f);
                rot.X += (window.MouseState.Delta.Y / 100f);

                rot.X = Math.Max(-1.5f, Math.Min(1.5f, rot.X));
            }

            // camera panning
            if (window.MouseState.IsButtonDown(MouseButton.Middle))
            {
                cameraPosition.X -= window.MouseState.Delta.X / 300f;
                cameraPosition.Y += window.MouseState.Delta.Y / 300f;

                UpdateViewMatrix();
            }
        }
    }
}
