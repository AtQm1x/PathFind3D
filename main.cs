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
    public enum DrawMode { Wireframe, Wall, Air, Start, End, Open, Closed, Path }
    public enum NodeState { Conductor, Dielectric, Start, End, Path }

    public class main
    {
        /* 1. доля частинок провідника в першій провідній жилі в мометн виникання та довжина цієї жили
         * 2. дисперсія кількості в жилі
         * 3. дисперсія електро провідності
         *
         *  діелектрик <> провідник
         *  поперечний розмір жили
         *  електропровідність ділянок
         *  монодисперсія
         *  
         *  
         *  
         *  
         *  
         */

        // не ураховувати на гранях
        // 
        double avgDispersion(params double[] vals)
        {
            double avg = vals.Average();
            double sum = 0;
            foreach (double item in vals)
            {
                sum += (item - avg) * (item - avg);
            }
            return Math.Sqrt(sum / vals.Length);
        }

        public main(int resX, int resY, string title)
        {
            this.resX = resX;
            this.resY = resY;
            this.title = title;
        }
        #region operators

        public static bool V3iLessThan(Vector3i v1, Vector3i v2)
        {
            return (v1.X < v2.X) && (v1.Y < v2.Y) && (v1.Z < v2.Z);
        }

        public static bool V3iGreaterThan(Vector3i v1, Vector3i v2)
        {
            return v1.X > v2.X && v1.Y > v2.Y && v1.Z > v2.Z;
        }

        #endregion

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
        private static Vector3i startNodePos = (0, 0, 0);
        private static Vector3i endNodePos = (gridSize.X, gridSize.Y, gridSize.Z);

        private bool exit = false;
        private static Matrix4 projectionMatrix;
        private static Matrix4 viewMatrix;
        private static float zoom = gridSize.X / 4;
        private Vector3 cameraPosition = new Vector3(0, 0, zoom);
        private GraphNode[,,] grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];
        private static Vector3 rot = new(0);

        private double obstacleDensity = 0.2;

        private bool continueSearch = true;
        private List<GraphNode> openSet = new();
        private SortedSet<GraphNode> openSetSorted = new();
        private List<GraphNode> closedSet = new();
        private GraphNode boxNode;
        private Vector3 baseSize;
        private bool drawWalls = true;


        List<GraphNode> AstarPath = new();
        List<GraphNode> BFSPath = new();

        // fps calculation variables
        private double elapsedTime = 0.0;
        private int frameCount = 0;


        Thread? BFSThread;
        Thread? AStarThread;


        static Vector3i[] mainDirections =
        {
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1)
        };

        static Vector3i[] directions = new Vector3i[]
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
            // update view matrix based on camera GridPosition
            viewMatrix = Matrix4.LookAt(
                cameraPosition,
                (cameraPosition.X, cameraPosition.Y, cameraPosition.Z - 1),
                Vector3.UnitY
            );
        }

        #endregion

        #region grid

        private void rebuildGrid()
        {
            // stop any ongoing search
            continueSearch = false;
            gridSize.X = Math.Max(gridSize.X, 1);
            gridSize.Y = Math.Max(gridSize.Y, 1);
            gridSize.Z = Math.Max(gridSize.Z, 1);
            startNodePos = Vector3i.Clamp(startNodePos, Vector3i.Zero, gridSize);
            endNodePos = Vector3i.Clamp(endNodePos, Vector3i.Zero, gridSize);
            Thread thread = new Thread(() =>
            {
                runRebuildGridThread(ref grid);
            });

            thread.Start();
            continueSearch = true;
        }

        int oRadius = 5;
        int oSpacing = 5;
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

                        GraphNode nd = new((i, j, k))
                        {
                            Rotation = rot,
                            AspectRatio = aspectRatio,
                            canGenerate = true
                        };

                        if (gridBuffer[i, j, k] == null)
                        {
                            gridBuffer[i, j, k] = nd;
                        }

                        if (gridBuffer[i, j, k].canGenerate == true)
                        {
                            // randomly set node as air or wall
                            if (rng.NextDouble() >= obstacleDensity)
                            {
                                gridBuffer[i, j, k].DrawMD = DrawMode.Air;
                                gridBuffer[i, j, k].State = NodeState.Conductor;
                                gridBuffer[i, j, k].canGenerate = true;
                            }
                            else
                            {
                                // sphere generation
                                Console.WriteLine($"trying to generate a sphere at {(i, j, k)}");
                                Parallel.For(-(oRadius + oSpacing), oRadius + oSpacing, (int ii) =>
                                {
                                    for (int jj = -(oRadius + oSpacing); jj < oRadius + oSpacing; jj++)
                                    {
                                        for (int kk = -(oRadius + oSpacing); kk < oRadius + oSpacing; kk++)
                                        {
                                            float len = new Vector3i(ii, jj, kk).EuclideanLength;
                                            Vector3i oPos = (i + ii, j + jj, k + kk);
                                            bool isInGrid = V3iGreaterThan(oPos, -Vector3i.One) && V3iLessThan(oPos, gridSize);

                                            if (len <= oRadius && isInGrid)
                                            {
                                                //if (gridBuffer[oPos.X, oPos.Y, oPos.Z] != null && gridBuffer[oPos.X, oPos.Y, oPos.Z].State == NodeState.Conductor)
                                                gridBuffer[oPos.X, oPos.Y, oPos.Z] = new GraphNode(oPos)
                                                {
                                                    DrawMD = DrawMode.Wall,
                                                    State = NodeState.Dielectric,
                                                    Rotation = rot,
                                                    AspectRatio = aspectRatio,
                                                    canGenerate = false
                                                };
                                            }
                                            else if (len <= oRadius * 2 + oSpacing && isInGrid)
                                            {
                                                gridBuffer[oPos.X, oPos.Y, oPos.Z] = new GraphNode(oPos)
                                                {
                                                    DrawMD = DrawMode.Air,
                                                    State = NodeState.Conductor,
                                                    Rotation = rot,
                                                    AspectRatio = aspectRatio,
                                                    canGenerate = false
                                                };
                                            }
                                        }
                                    }
                                });
                            }
                        }

                        // set start and end nodes
                        if (i == startNodePos.X && j == startNodePos.Y && k == startNodePos.Z)
                            nd.DrawMD = DrawMode.Start;

                        if (i == endNodePos.X - 1 && j == endNodePos.Y - 1 && k == endNodePos.Z - 1)
                            nd.DrawMD = DrawMode.End;
                    };
                };
            };


            grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];
            grid = gridBuffer;
            updateMesh = true;
        }

        private void drawGrid()
        {
            // sort transparent objects for correct rendering

            GL.UseProgram(shader.Handle);
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
        Vector3i[] usedDirections = directions;

        #region BFS
        private void AddNeighborsToOpenSet(GraphNode currentNode)
        {
            Vector3i nodePos = currentNode.GridPosition;
            foreach (var direction in directions)
            {
                Vector3i newNodePos = nodePos + direction;

                // check if new GridPosition is within grid bounds
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

                // check if new GridPosition is within grid bounds
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

            GraphNode startNode = grid[startNodePos.X, startNodePos.Y, startNodePos.Z];

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

        private void updateMousePos()
        {
            // check if mouse is over the ImGui window
            NVector2 windowPos = ImGui.GetWindowPos();
            NVector2 windowSize = ImGui.GetWindowSize();

            _isMouseOverMenu |= (mousePos.X >= windowPos.X &&
                               mousePos.X <= windowPos.X + windowSize.X &&
                               mousePos.Y >= windowPos.Y &&
                               mousePos.Y <= windowPos.Y + windowSize.Y);
        }

        NVector2 mousePos;
        bool _input_grid_size = false;
        bool _isMouseOverMenu = false;

        int gsX = gridSize.X, gsY = gridSize.Y, gsZ = gridSize.Z, snX = startNodePos.X, snY = startNodePos.Y, snZ = startNodePos.Z, enX = endNodePos.X, enY = endNodePos.Y, enZ = endNodePos.Z;
        private void ProcessGUI()
        {
            ImGui.DockSpaceOverViewport(ImGui.GetMainViewport().ID, ImGui.GetMainViewport(), ImGuiDockNodeFlags.PassthruCentralNode);

            ImGui.Begin("Control");
            ImGui.SetWindowFontScale(1.2f);
            _isMouseOverMenu = false;
            mousePos = ImGui.GetMousePos();
            updateMousePos();
            //  if (ImGui.Button("test deviation")) Console.WriteLine(avgDeviation(4, 8, 6, 5, 3, 7));

            // gui buttons for various actions
            if (ImGui.Button("Rebuild Grid"))
            {
                rebuildGrid();
            }

            if (ImGui.Button(drawWalls == true ? "Hide Walls" : "UnHide Walls"))
            {
                drawWalls = !drawWalls;
            }

            if (ImGui.Button("change draw modes"))
            {

            }

            if (ImGui.Button("Clear Path"))
            {
                clearPath();
            }

            if (ImGui.Button("run BFS"))
            {
                if (grid[startNodePos.X, startNodePos.Y, startNodePos.Z] != null && grid[endNodePos.X - 1, endNodePos.Y - 1, endNodePos.Z - 1] != null)
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

            if (ImGui.Button(_input_grid_size == false ? "Open Config" : "Close Config"))
            {
                _input_grid_size = !_input_grid_size;
            }

            if (_input_grid_size)
            {
                ImGui.Begin("Config");
                updateMousePos();

                ImGui.Text("Grid Size");
                ImGui.InputInt("X", ref gsX);
                ImGui.InputInt("Y", ref gsY);
                ImGui.InputInt("Z", ref gsZ);

                ImGui.Text("Start Node");
                ImGui.InputInt("X ", ref snX);
                ImGui.InputInt("Y ", ref snY);
                ImGui.InputInt("Z ", ref snZ);

                ImGui.Text("End Node");
                ImGui.InputInt("X  ", ref enX);
                ImGui.InputInt("Y  ", ref enY);
                ImGui.InputInt("Z  ", ref enZ);

                ImGui.Text("Obstacle Density");
                ImGui.InputDouble(" ", ref obstacleDensity, 0.05);

                ImGui.Text("Dielectric Particle size");
                ImGui.InputInt("Radius", ref oRadius);
                ImGui.InputInt("Minimal spacing", ref oSpacing);

                if (ImGui.Button("Apply"))
                {
                    gridSize = new(gsX, gsY, gsZ);
                    startNodePos = new(snX, snY, snZ);
                    endNodePos = new(enX, enY, enZ);

                    snX = Math.Clamp(snX, 0, gsX);
                    snY = Math.Clamp(snY, 0, gsY);
                    snZ = Math.Clamp(snZ, 0, gsZ);

                    enX = Math.Clamp(enX, 0, gsX);
                    enY = Math.Clamp(enY, 0, gsY);
                    enZ = Math.Clamp(enZ, 0, gsZ);

                    obstacleDensity = Math.Clamp(obstacleDensity, 0, 1);

                    rebuildGrid();
                }
            }

            ImGui.End();
        }

        #endregion

        #region shaders
        private static Shader shader;

        private static string vertexShaderSource = @"
            #version 330 core
            layout (location = 0) in vec3 aPos;
            layout (location = 1) in vec4 aColor;
            
            uniform mat4 model;
            uniform mat4 view;
            uniform mat4 projection;
            
            out vec4 vertexColor;
            
            void main()
            {
                gl_Position = projection * view * model * vec4(aPos, 1.0);
                vertexColor = aColor;
            }
        ";

        private static string fragmentShaderSource = @"
            #version 330 core
            in vec4 vertexColor;
            out vec4 FragColor;
            uniform vec4 cColor;
            
            void main()
            {
                if (cColor == vec4(0,0,0,0))
                    FragColor = vertexColor;
                    
                else
                    FragColor = cColor;
            }
        ";
        #endregion

        #region Mesh
        private static int vbo, cbo, ebo, vao;
        private static (Vector3[] vertices, uint[] indices, Vector4[] colors) meshData;

        private void updateVBOs()
        {
            // Generate and bind the Vertex Array Object
            GL.GenVertexArrays(1, out vao);
            GL.BindVertexArray(vao);

            // Vertex buffer
            GL.GenBuffers(1, out vbo);
            GL.BindBuffer(BufferTarget.ArrayBuffer, vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, meshData.vertices.Length * Vector3.SizeInBytes, meshData.vertices, BufferUsageHint.StaticDraw);
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, Vector3.SizeInBytes, 0);

            // Color buffer
            GL.GenBuffers(1, out cbo);
            GL.BindBuffer(BufferTarget.ArrayBuffer, cbo);
            GL.BindBuffer(BufferTarget.ArrayBuffer, cbo);
            GL.BufferData(BufferTarget.ArrayBuffer, meshData.colors.Length * Vector4.SizeInBytes, meshData.colors, BufferUsageHint.StaticDraw);
            GL.EnableVertexAttribArray(1);
            GL.VertexAttribPointer(1, 4, VertexAttribPointerType.Float, false, Vector4.SizeInBytes, 0);

            // Element buffer
            GL.GenBuffers(1, out ebo);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, ebo);
            GL.BufferData(BufferTarget.ElementArrayBuffer, meshData.indices.Length * sizeof(uint), meshData.indices, BufferUsageHint.StaticDraw);

            GL.BindVertexArray(0);
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
            // Delete OpenGL objects
            GL.DeleteBuffer(vbo);
            GL.DeleteBuffer(cbo);
            GL.DeleteBuffer(ebo);
            GL.DeleteVertexArray(vao);

            // Reset identifiers
            vbo = cbo = ebo = vao = 0;

            // Clear mesh data
            if (meshData.vertices != null) meshData.vertices = new Vector3[] { new Vector3(0, 0, 0) };
            if (meshData.indices != null) meshData.indices = new uint[] { 0 };
            if (meshData.colors != null) meshData.colors = new Vector4[] { new Vector4(0) };

            // Reset meshData
            meshData = default;

            GC.Collect();

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
            shader = new Shader(vertexShaderSource, fragmentShaderSource);

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

        private VoxelMesher mesher;
        private bool updateMesh = true;
        private void onUpdateFrame(FrameEventArgs e)
        {
            if (window == null)
                return;

            updateGrid();

            if (updateMesh)
            {
                meshData = default;
                updateMesh = false;
                mesher = new(grid, gridSize);
                meshData = mesher.GenerateMesh();
                updateVBOs();
            }
            // update box if necessary
            if ((boxNode?.Rotation != rot || boxNode?.AspectRatio != aspectRatio))
            {
                if (boxNode != null)
                {
                    boxNode.Rotation = rot;
                    boxNode.AspectRatio = aspectRatio;
                }
            }


            calculateFPS(e.Time);
            handleUserInput();

            runAction?.Invoke();
        }

        bool chngDielectricAndConductor;
        private static void useMeshRender()
        {
            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.CullFace);

            shader.Use();

            // Create rotation matrix
            Matrix4 rotationX = Matrix4.CreateRotationX(rot.X);
            Matrix4 rotationY = Matrix4.CreateRotationY(rot.Y);
            Matrix4 rotationZ = Matrix4.CreateRotationZ(rot.Z);
            Matrix4 rotation = rotationZ * rotationY * rotationX;

            // Apply rotation to model matrix
            Matrix4 model = rotation;

            shader.SetMatrix4("model", model);
            shader.SetMatrix4("view", viewMatrix);
            shader.SetMatrix4("projection", projectionMatrix);
            shader.SetVec4("cColor", (0, 0, 0, 0));

            GL.BindVertexArray(vao);

            // Draw filled cubes
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            GL.DrawElements(PrimitiveType.Triangles, meshData.indices.Length, DrawElementsType.UnsignedInt, 0);

            // Draw wireframe
            GL.LineWidth(2.0f);
            shader.SetVec4("cColor", (0, 0, 0, 1));
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
            GL.DrawElements(PrimitiveType.Triangles, meshData.indices.Length, DrawElementsType.UnsignedInt, 0);

            GL.BindVertexArray(0);

            GL.Disable(EnableCap.DepthTest);
            GL.Disable(EnableCap.CullFace);

        }
        private void onRenderFrame(FrameEventArgs args)
        {
            if (window == null || grid == null)
                return;

            GL.ClearColor(0.2f, 0.3f, 0.4f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            GL.Viewport(0, 0, window.ClientSize.X, window.ClientSize.Y);

            drawGrid();

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
