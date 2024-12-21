using ImGuiNET;
using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using NVector2 = System.Numerics.Vector2;

namespace PathFind3D
{
    // enum for different rendering modes
    public enum DrawMode { Wireframe, Wall, Air, Start, End, Open, Closed, Path }
    public enum NodeState { Conductor, Dielectric, Start, End, Path }

    public class main
    {
        /*
         *  XY   доля частинок провідника в першій провідній жилі в мометн виникання та довжина цієї жили
         *  X    дисперсія кількості в жилі
         *  X    дисперсія електро провідності
         *  Y   діелектрик <> провідник 
         *  X   поперечний розмір жили 
         *  X   електропровідність ділянок 
         *  X   монодисперсія 
         */

        /*
            ПРЕЗЕНТАЦІЯ
            Титульна Сторінка на якій буде тема
            1)Актуальність 2)Мета
            Основні задачі
            засоби розв'язування задач

            {
            Технологія визначення довжини шляху
            Технологія визначення кількості частинок в ньому
            
            кількість відгалужень

            дисперсія і середнє значення кількості частинок в тупіках

            відношення середнього до загальної кількості частинок
            }
        */

        // набрати статистику



        //  кількість частинок у тупіках

        // опис програми = задачі, результати, інструкція

        double avgDispersion(GraphNode[,,] vals)
        {
            double sum = 0;
            int c = 0;
            foreach (var node in vals)
            {
                c++;
                if (node.State == NodeState.Conductor)
                {
                    sum++;
                }
                else
                {
                    sum--;
                }
            }

            double avg = sum / c;
            double sum2 = 0;
            foreach (var node in vals)
            {
                if (node.State == NodeState.Conductor)
                {
                    sum2 += Math.Pow(1 - avg, 2);
                }
                else
                {
                    sum2 += Math.Pow(-1 - avg, 2);
                }
            }
            return Math.Round(1 - Math.Sqrt(sum2 / vals.Length), 10);
        }

        double getFractionOfParticlesInPath()
        {
            int total = gridSize.X * gridSize.Y * gridSize.Z;
            int countOfPathNodes = 0;
            lock (grid)
            {
                _infoMenu_isOpen = false;
                GraphNode[] castGrid = grid.Cast<GraphNode>().ToArray();
                for (int i = 0; i < castGrid.Count(); i++)
                {
                    GraphNode item = castGrid[i];
                }
                _infoMenu_isOpen = true;
            }
            return (double)countOfPathNodes / total;
        }
        // визначити вплив відношення розмірів частинки провідника і всього виробу на можливість застосування теорії перколяції
        double getConductorConcentration()
        {
            int tcount = gridSize.X * gridSize.Y * gridSize.Z;
            int count = 0;
            if (grid == null)
            {
                return 0;
            }
            lock (this)
            {
                foreach (var item in grid)
                {
                    if (item == null)
                    {
                        return 0;
                    }
                    if (item.State == NodeState.Conductor)
                    {
                        count++;
                    }
                }
            }
            return count / (double)tcount;
        }

        public main(int resX, int resY, string title)
        {
            this.resX = resX;
            this.resY = resY;
            this.title = title;
        }

        #region operators

        public static bool V3iLessThanOr(Vector3i v1, Vector3i v2)
        {
            return (v1.X < v2.X) || (v1.Y < v2.Y) || (v1.Z < v2.Z);
        }

        public static bool V3iGreaterThanOr(Vector3i v1, Vector3i v2)
        {
            return v1.X > v2.X || v1.Y > v2.Y || v1.Z > v2.Z;
        }
        public static bool V3iLessThanAnd(Vector3i v1, Vector3i v2)
        {
            return (v1.X < v2.X) && (v1.Y < v2.Y) && (v1.Z < v2.Z);
        }

        public static bool V3iGreaterThanAnd(Vector3i v1, Vector3i v2)
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

        private static Vector3i gridSize = (32, 1, 32);
        private static Vector3i startNodePos = (gridSize.X / 2, gridSize.Y / 2, 0);
        private static Vector3i endNodePos = (gridSize.X / 2, gridSize.Y / 2, gridSize.Z);

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
        private static bool drawWalls = true;


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
            new Vector3i(-1, -1, 1), new Vector3i(-1, -1, 0), new Vector3i(-1, -1, -1)
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

        private void rebuildGrid_Complex()
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
                runRebuildGridThread_Complex(ref grid);
            });

            thread.Start();
            continueSearch = true;
        }
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

        static int oRadius = 3;
        static int oSpacing = 1;
        int criteria = oRadius + oSpacing + 1;
        private void runRebuildGridThread_Complex(ref GraphNode[,,] grid)
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
                            canGenerate = true,
                            State = NodeState.Dielectric,
                            DrawMD = DrawMode.Air
                        };
                        gridBuffer[i, j, k] = nd;
                    }
                }
            }

            // populate grid with nodes
            for (int i = 0; i < gridSize.X; i++)
            {
                for (int j = 0; j < gridSize.Y; j++)
                {
                    for (int k = 0; k < gridSize.Z; k++)
                    {
                        if (gridBuffer[i, j, k].canGenerate)
                        {
                            // randomly set node as air or wall
                            double rng_d = rng.NextDouble();
                            if (rng_d >= obstacleDensity)
                            {
                                logger.WriteLine(rng_d);
                                gridBuffer[i, j, k].DrawMD = DrawMode.Air;
                                gridBuffer[i, j, k].State = NodeState.Dielectric;
                                gridBuffer[i, j, k].canGenerate = true;
                            }
                            else if (gridBuffer[i, j, k].canGenerate)
                            {
                                // sphere generation
                                for (int ii = -criteria; ii < criteria; ii++)
                                {
                                    for (int jj = -criteria; jj < criteria; jj++)
                                    {
                                        for (int kk = -criteria; kk < criteria; kk++)
                                        {
                                            float len = new Vector3i(ii, jj, kk).EuclideanLength;
                                            Vector3i oPos = (i + ii, j + jj, k + kk);

                                            bool isInGrid = V3iGreaterThanOr(oPos, -Vector3i.One) && V3iLessThanOr(oPos, gridSize);

                                            if (isInGrid)
                                            {
                                                if (len <= oRadius * 2 + oSpacing * 2)
                                                {
                                                    gridBuffer[oPos.X, oPos.Y, oPos.Z].canGenerate = false;
                                                }
                                                if (len <= oRadius)
                                                {
                                                    gridBuffer[oPos.X, oPos.Y, oPos.Z].DrawMD = DrawMode.Wall;
                                                    gridBuffer[oPos.X, oPos.Y, oPos.Z].State = NodeState.Conductor;
                                                    gridBuffer[oPos.X, oPos.Y, oPos.Z].canGenerate = false;
                                                }
                                            }
                                        }
                                    }
                                };
                            }
                        }
                    };
                };
            };


            // set start and end nodes
            for (int ii = -criteria; ii < criteria; ii++)
            {
                for (int jj = -criteria; jj < criteria; jj++)
                {
                    for (int kk = -criteria; kk < criteria; kk++)
                    {
                        //if (ii == 0 && jj == 0 && kk == 0) continue;
                        float len = new Vector3i(ii, jj, kk).EuclideanLength;
                        Vector3i oPosStart = (startNodePos.X + ii, startNodePos.Y + jj, startNodePos.Z + kk);
                        Vector3i oPosEnd = (endNodePos.X + ii, endNodePos.Y + jj, endNodePos.Z + kk);

                        bool isInGridStart = V3iGreaterThanOr(oPosStart, -Vector3i.One) && V3iLessThanOr(oPosStart, gridSize);
                        bool isInGridEnd = V3iGreaterThanOr(oPosEnd, -Vector3i.One) && V3iLessThanOr(oPosEnd, gridSize);

                        if (isInGridStart && len < oRadius)
                        {
                            gridBuffer[oPosStart.X, oPosStart.Y, oPosStart.Z].DrawMD = DrawMode.Wall;
                            gridBuffer[oPosStart.X, oPosStart.Y, oPosStart.Z].State = NodeState.Conductor;
                            gridBuffer[oPosStart.X, oPosStart.Y, oPosStart.Z].canGenerate = false;
                        }
                        if (isInGridEnd && len < oRadius)
                        {
                            gridBuffer[oPosEnd.X, oPosEnd.Y, oPosEnd.Z].DrawMD = DrawMode.Wall;
                            gridBuffer[oPosEnd.X, oPosEnd.Y, oPosEnd.Z].State = NodeState.Conductor;
                            gridBuffer[oPosEnd.X, oPosEnd.Y, oPosEnd.Z].canGenerate = false;
                        }
                    }
                }
            }
            startNodePos = new Vector3i(Math.Clamp(startNodePos.X, 0, gridSize.X - 1), Math.Clamp(startNodePos.Y, 0, gridSize.Y - 1), Math.Clamp(startNodePos.Z, 0, gridSize.Z - 1));
            endNodePos = new Vector3i(Math.Clamp(endNodePos.X, 0, gridSize.X - 1), Math.Clamp(endNodePos.Y, 0, gridSize.Y - 1), Math.Clamp(endNodePos.Z, 0, gridSize.Z - 1));

            gridBuffer[startNodePos.X, startNodePos.Y, startNodePos.Z].State = NodeState.Start;
            gridBuffer[startNodePos.X, startNodePos.Y, startNodePos.Z].DrawMD = DrawMode.Start;
            gridBuffer[endNodePos.X, endNodePos.Y, endNodePos.Z].State = NodeState.End;
            gridBuffer[endNodePos.X, endNodePos.Y, endNodePos.Z].DrawMD = DrawMode.End;
            grid = new GraphNode[gridSize.X, gridSize.Y, gridSize.Z];
            grid = gridBuffer;
            updateMesh = true;
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

                        if (gridBuffer[i, j, k].canGenerate)
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
                                gridBuffer[i, j, k].DrawMD = DrawMode.Wall;
                                gridBuffer[i, j, k].State = NodeState.Dielectric;
                                gridBuffer[i, j, k].canGenerate = true;
                            }
                        }
                    };
                };
            };


            // set start and end nodes
            startNodePos = V3IClampToGrid(startNodePos);
            endNodePos = V3IClampToGrid(endNodePos);

            gridBuffer[startNodePos.X, startNodePos.Y, startNodePos.Z].DrawMD = DrawMode.Start;
            gridBuffer[startNodePos.X, startNodePos.Y, startNodePos.Z].State = NodeState.Start;

            gridBuffer[endNodePos.X, endNodePos.Y, endNodePos.Z].DrawMD = DrawMode.End;
            gridBuffer[endNodePos.X, endNodePos.Y, endNodePos.Z].State = NodeState.End;

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
                        node.Draw(viewMatrix, projectionMatrix, shader, 36, vao[0]);
                }
            }

            // draw transparent objects
            foreach (var node in sortedGrid)
            {
                if (node != null)
                {
                    node?.Draw(viewMatrix, projectionMatrix, shader, 36, vao[1]);
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

        private Vector3i V3IClamp(Vector3i vec, Vector3i min, Vector3i max)
        {
            return new Vector3i(Math.Clamp(vec.X, min.X, max.X), Math.Clamp(vec.Y, min.Y, max.Y), Math.Clamp(vec.Z, min.Z, max.Z));
        }
        private Vector3i V3IClampToGrid(Vector3i vec)
        {
            return V3IClamp(vec, Vector3i.Zero, gridSize - Vector3i.One);
            return new Vector3i(Math.Clamp(vec.X, 0, gridSize.X - 1), Math.Clamp(vec.Y, 0, gridSize.Y - 1), Math.Clamp(vec.Z, 0, gridSize.Z - 1));
        }

        #endregion

        #region pathfinding

        Vector3i[] usedDirections = directions;
        #region BFS
        private void AddNeighborsToOpenSet(GraphNode currentNode)
        {
            Vector3i nodePos = currentNode.GridPosition;
            foreach (var direction in usedDirections)
            {
                Vector3i newNodePos = nodePos + direction;

                // check if new GridPosition is within grid bounds

                //logger.WriteLine($"{V3iLessThan(newNodePos, Vector3i.Zero)} || {V3iGreaterThan(newNodePos, gridSize - Vector3i.One)}");

                if (V3iLessThanOr(newNodePos, Vector3i.Zero) || V3iGreaterThanOr(newNodePos, gridSize - Vector3i.One))
                {
                    //logger.WriteLine($"skipped {newNodePos}");
                    continue;
                }
                try
                {
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

                    // check if end node is reached
                    if (neighbor.State == NodeState.End)
                    {
                        neighbor.Parent = currentNode;
                        continueSearch = false;
                        openSet.Clear();
                        closedSet.Clear();
                        logger.WriteLine("PathFound");
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
                    if (neighbor.State == NodeState.Conductor)
                    {
                        openSet.Add(neighbor);
                    }
                }
                catch (Exception)
                {

                    throw new Exception($"{(newNodePos.X, newNodePos.Y, newNodePos.Z)}");
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
                    if (node.DrawMD != DrawMode.Closed && node.State != NodeState.Start && node.State != NodeState.End)
                    {
                        node.DrawMD = DrawMode.Closed;
                    }
                }

                AddNeighborsToOpenSet(currentNode);
            }

            stopwatch.Stop();
            TimeSpan elapsedTime = stopwatch.Elapsed;
            logger.WriteLine($"BreadthFirstSearch execution time: {elapsedTime.TotalMilliseconds} ms");
            updateMesh = true;
            updateVBOs();
        }
        #endregion

        #region A*
        private void AddNeighborsToOpenSortedSet(GraphNode currentNode, Vector3i endNodePos, PriorityQueue<GraphNode, float> openSet, HashSet<GraphNode> closedSet)
        {
            foreach (Vector3i direction in usedDirections)
            {
                Vector3i newNodePos = currentNode.GridPosition + direction;

                // check if new GridPosition is within grid bounds
                if (V3iLessThanOr(newNodePos, Vector3i.Zero) || V3iGreaterThanOr(newNodePos, gridSize - Vector3i.One))
                {
                    continue;
                }

                GraphNode neighbor = grid[newNodePos.X, newNodePos.Y, newNodePos.Z];

                // skip if neighbor is in closed set or is a wall
                if (closedSet.Contains(neighbor) || neighbor.State == NodeState.Dielectric)
                {
                    continue;
                }

                float GScore = currentNode.gScore + direction.EuclideanLength;

                // update node if better path found
                if (!openSet.UnorderedItems.Any(x => x.Element == neighbor) || GScore < neighbor.gScore)
                {
                    neighbor.Parent = currentNode;
                    neighbor.gScore = GScore;
                    neighbor.hScore = (neighbor.GridPosition - endNodePos).EuclideanLengthSquared;
                    neighbor.fScore = neighbor.gScore + neighbor.hScore;

                    if (!openSet.UnorderedItems.Any(x => x.Element == neighbor))
                    {
                        openSet.Enqueue(neighbor, neighbor.fScore);
                        if (neighbor.State != NodeState.Start && neighbor.State != NodeState.End)
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

        public bool AStar(Vector3i startPos, Vector3i endPos, bool updateGrid = true, bool logToFile = true)
        {
            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            PriorityQueue<GraphNode, float> openSet = new PriorityQueue<GraphNode, float>();
            HashSet<GraphNode> closedSet = new HashSet<GraphNode>();

            GraphNode startNode = grid[startPos.X, startPos.Y, startPos.Z];

            // initialize start node
            startNode.gScore = 0;
            startNode.hScore = (startPos - endPos).EuclideanLengthSquared;
            startNode.fScore = startNode.hScore;
            openSet.Enqueue(startNode, startNode.fScore);

            while (openSet.Count > 0)
            {
                GraphNode currentNode = openSet.Dequeue();

                // check if reached the end node
                if (currentNode.GridPosition == endPos)
                {
                    if (logToFile) logger.WriteLine("Path found!");
                    BacktrackPath(currentNode);
                    break;
                }

                closedSet.Add(currentNode);

                lock (grid)
                {
                    // update node color for visualization
                    if (currentNode.GridPosition != startPos && (currentNode.GridPosition != endPos || currentNode.DrawMD != DrawMode.End))
                    {
                        currentNode.DrawMD = DrawMode.Closed;
                    }
                    AddNeighborsToOpenSortedSet(currentNode, endPos, openSet, closedSet);
                }
            }

            // no path found
            TimeSpan elapsedTime = stopwatch.Elapsed;
            stopwatch.Stop();
            if (openSet.Count == 0)
            {
                return false;
            }
            if (logToFile) logger.WriteLine($"A* execution time: {elapsedTime.TotalMilliseconds} ms");
            updateMesh = updateGrid;
            return true;
        }
        #endregion

        private void BacktrackPath(GraphNode endNode)
        {
            int PathLen = 0;
            GraphNode currentNode = endNode;

            // trace path from end to start
            while (currentNode.Parent != null && currentNode.State != NodeState.Start)
            {
                currentNode = currentNode.Parent;
                if (currentNode.State != NodeState.Start)
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
            logger.WriteLine($"Path length: {MathF.Round(endNode.gScore, 3) - 1}");
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
        bool _configMenu_isOpen = false;
        bool _infoMenu_isOpen = false;
        bool _isMouseOverMenu = false;

        int gsX = gridSize.X, gsY = gridSize.Y, gsZ = gridSize.Z, snX = startNodePos.X, snY = startNodePos.Y, snZ = startNodePos.Z, enX = endNodePos.X, enY = endNodePos.Y, enZ = endNodePos.Z;

        Thread testThread;

        private void ProcessGUI()
        {
            ImGui.DockSpaceOverViewport(ImGui.GetMainViewport().ID, ImGui.GetMainViewport(), ImGuiDockNodeFlags.PassthruCentralNode);

            ImGui.Begin("Control");
            ImGui.SetWindowFontScale(1.2f);

            _isMouseOverMenu = false;
            mousePos = ImGui.GetMousePos();
            updateMousePos();

            if (ImGui.Button("test deviation"))
            {
                logger.WriteLine(avgDispersion(grid));
            }

            if (ImGui.Button("Rebuild Grid"))
            {
                rebuildGrid();
            }

            if (ImGui.Button("test can percolate"))
            {
                testThread = new Thread(() =>
                {
                    logger.WriteLine(percolationThreshold());
                });

                testThread.Start();
            }

            if (ImGui.Button(drawWalls == true ? "Hide Walls" : "UnHide Walls"))
            {
                drawWalls = !drawWalls;
            }

            if (ImGui.Button("change draw modes"))
            {
                chngDielectricAndConductor = !chngDielectricAndConductor;
                updateMesh = true;
            }

            if (ImGui.Button("Clear Path"))
            {
                clearPath();
            }

            if (ImGui.Button("run BFS"))
            {
                startNodePos = V3IClampToGrid(startNodePos);
                endNodePos = V3IClampToGrid(endNodePos);
                if (grid[startNodePos.X, startNodePos.Y, startNodePos.Z] != null && grid[endNodePos.X, endNodePos.Y, endNodePos.Z] != null)
                {
                    grid[startNodePos.X, startNodePos.Y, startNodePos.Z].DrawMD = DrawMode.Start;
                    grid[endNodePos.X, endNodePos.Y, endNodePos.Z].DrawMD = DrawMode.End;
                    BFSThread = new Thread(() =>
                    {
                        BreadthFirstSearch(grid[startNodePos.X, startNodePos.Y, startNodePos.Z]);
                    });

                    BFSThread.Start();
                }
            }

            if (ImGui.Button("run A*"))
            {

                // clamp start and end node positions
                startNodePos = V3IClampToGrid(startNodePos);
                endNodePos = V3IClampToGrid(endNodePos);

                grid[startNodePos.X, startNodePos.Y, startNodePos.Z].DrawMD = DrawMode.Start;
                grid[endNodePos.X, endNodePos.Y, endNodePos.Z].DrawMD = DrawMode.End;

                AStarThread = new Thread(() =>
                {
                    AStar(startNodePos, endNodePos);
                });

                AStarThread.Start();
            }

            if (ImGui.Button(_configMenu_isOpen == false ? "Open Config" : "Close Config"))
            {
                _configMenu_isOpen = !_configMenu_isOpen;
            }

            if (ImGui.Button(_infoMenu_isOpen == false ? "Open Info" : "Close Info"))
            {
                _infoMenu_isOpen = !_infoMenu_isOpen;
            }

            if (_configMenu_isOpen)
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

                    startNodePos = (Math.Clamp(startNodePos.X, 0, gridSize.X - 1), Math.Clamp(startNodePos.Y, 0, gridSize.Y - 1), Math.Clamp(startNodePos.Z, 0, gridSize.Z - 1));
                    endNodePos = (Math.Clamp(endNodePos.X, 0, gridSize.X - 1), Math.Clamp(endNodePos.Y, 0, gridSize.Y - 1), Math.Clamp(endNodePos.Z, 0, gridSize.Z - 1));

                    obstacleDensity = Math.Clamp(obstacleDensity, 0, 1);

                    rebuildGrid();
                }
            }

            if (_infoMenu_isOpen)
            {
                ImGui.Begin("Info");
                ImGui.SetWindowFontScale(1.2f);
                ImGui.Text($"Concentration: {getConductorConcentration()}");
                ImGui.Text($"Fraction of particles in the path: {getFractionOfParticlesInPath()}");
                updateMousePos();
            }

            ImGui.Begin("Program Log");
            updateMousePos();

            try
            {
                using (var stream = new FileStream(_LogFilePath, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
                using (var reader = new StreamReader(stream))
                {
                    fileContent = reader.ReadToEnd();
                }
            }
            catch (IOException ex)
            {
                fileContent = "Error reading log file: " + ex.Message;
            }

            ImGui.InputTextMultiline("Program Log", ref fileContent, (uint)fileContent.Length + 1, new NVector2(-1, -1), ImGuiInputTextFlags.ReadOnly);
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

        static uint _bufferCount = 3;

        private static int[]
            vbo = new int[_bufferCount],
            cbo = new int[_bufferCount],
            ebo = new int[_bufferCount],
            vao = new int[_bufferCount];

        private static
            (Vector3[] vertices, uint[] indices, Vector4[] colors)[] meshData
                = new (Vector3[] vertices, uint[] indices, Vector4[] colors)[_bufferCount];
        private void DisposeMeshData()
        {
            for (int i = 0; i < _bufferCount; i++)
            {
                if (meshData[i].vertices != null)
                {
                    Array.Clear(meshData[i].vertices, 0, meshData[i].vertices.Length);
                    meshData[i].vertices = null;
                }

                if (meshData[i].indices != null)
                {
                    Array.Clear(meshData[i].indices, 0, meshData[i].indices.Length);
                    meshData[i].indices = null;
                }

                if (meshData[i].colors != null)
                {
                    Array.Clear(meshData[i].colors, 0, meshData[i].colors.Length);
                    meshData[i].colors = null;
                }

                // Reset the struct to its default values
                meshData[i] = default;
            }
        }

        private void updateVBOs()
        {
            for (int i = 0; i < _bufferCount; i++)
            {
                // Generate and bind the Vertex Array Object
                GL.GenVertexArrays(1, out vao[i]);
                GL.BindVertexArray(vao[i]);

                // Vertex buffer
                GL.GenBuffers(1, out vbo[i]);
                GL.BindBuffer(BufferTarget.ArrayBuffer, vbo[i]);
                GL.BufferData(BufferTarget.ArrayBuffer, meshData[i].vertices.Length * Vector3.SizeInBytes, meshData[i].vertices, BufferUsageHint.StaticDraw);
                GL.EnableVertexAttribArray(0);
                GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, Vector3.SizeInBytes, 0);

                // Color buffer
                GL.GenBuffers(1, out cbo[i]);
                GL.BindBuffer(BufferTarget.ArrayBuffer, cbo[i]);
                GL.BindBuffer(BufferTarget.ArrayBuffer, cbo[i]);
                GL.BufferData(BufferTarget.ArrayBuffer, meshData[i].colors.Length * Vector4.SizeInBytes, meshData[i].colors, BufferUsageHint.StaticDraw);
                GL.EnableVertexAttribArray(1);
                GL.VertexAttribPointer(1, 4, VertexAttribPointerType.Float, false, Vector4.SizeInBytes, 0);

                // Element buffer
                GL.GenBuffers(1, out ebo[i]);
                GL.BindBuffer(BufferTarget.ElementArrayBuffer, ebo[i]);
                GL.BufferData(BufferTarget.ElementArrayBuffer, meshData[i].indices.Length * sizeof(uint), meshData[i].indices, BufferUsageHint.StaticDraw);

                GL.BindVertexArray(0);
            }
        }
        #endregion

        #region log File

        public const string _LogFilePath = "programLog.txt";
        FileLogger logger = new FileLogger(_LogFilePath);
        string fileContent = string.Empty;

        private void fileINIT()
        {
            if (File.Exists(_LogFilePath))
            {
                File.WriteAllText(_LogFilePath, string.Empty);
            }
            else
            {
                File.Create(_LogFilePath).Close();
            }
            fileContent = File.ReadAllText(_LogFilePath);
        }

        #endregion

        #region fizykaaaaaaaaaaaaa

        private double percolationThreshold()
        {
            bool pathFound = false;
            for (int i = 0; i < gridSize.X; i++)
            {
                for (int j = 0; j < gridSize.Y; j++)
                {
                    if (grid[i, j, 0].State != NodeState.Conductor)
                        continue;
                    for (int k = 0; k < gridSize.X; k++)
                    {
                        for (int l = 0; l < gridSize.Y; l++)
                        {
                            if (grid[k, l, gridSize.Z - 1].State != NodeState.Conductor)
                                continue;
                            if (AStar((i, j, 0), (k, l, gridSize.Z - 1), false, false))
                            {
                                pathFound = true;
                                goto exitLoop;
                            }
                            clearPath(false);
                        }
                    }
                }
            }
        exitLoop:
            updateMesh = true;
            return pathFound ? 1 : 0;
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
                Vector3i newstartNodePos = new Vector3i(Math.Clamp(startNodePos.X, 0, gridSize.X - 1), Math.Clamp(startNodePos.Y, 0, gridSize.Y - 1), Math.Clamp(startNodePos.Z, 0, gridSize.Z - 1));
                Vector3i newendNodePos = new Vector3i(Math.Clamp(endNodePos.X, 0, gridSize.X - 1), Math.Clamp(endNodePos.Y, 0, gridSize.Y - 1), Math.Clamp(endNodePos.Z, 0, gridSize.Z - 1));
                grid[newstartNodePos.X, newstartNodePos.Y, newstartNodePos.Z].DrawMD = DrawMode.Start;
                grid[newendNodePos.X, newendNodePos.Y, newendNodePos.Z].DrawMD = DrawMode.End;
                BFSThread = new Thread(() =>
                {
                    BreadthFirstSearch(grid[newstartNodePos.X, newstartNodePos.Y, newstartNodePos.Z]);
                });

                BFSThread.Start();
            }

            if (window.KeyboardState.IsKeyPressed(Keys.G))
            {
                Vector3i newstartNodePos = new Vector3i(Math.Clamp(startNodePos.X, 0, gridSize.X - 1), Math.Clamp(startNodePos.Y, 0, gridSize.Y - 1), Math.Clamp(startNodePos.Z, 0, gridSize.Z - 1));
                Vector3i newendNodePos = new Vector3i(Math.Clamp(endNodePos.X, 0, gridSize.X - 1), Math.Clamp(endNodePos.Y, 0, gridSize.Y - 1), Math.Clamp(endNodePos.Z, 0, gridSize.Z - 1));
                grid[newstartNodePos.X, newstartNodePos.Y, newstartNodePos.Z].DrawMD = DrawMode.Start;
                grid[newendNodePos.X, newendNodePos.Y, newendNodePos.Z].DrawMD = DrawMode.End;
                AStarThread = new Thread(() =>
                {
                    AStar(newstartNodePos, newendNodePos);
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

        private void clearPath(bool doUpdateGrid = true)
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
            updateMesh = doUpdateGrid;
            continueSearch = true;
        }

        public void Shutdown()
        {
            for (int i = 0; i < _bufferCount; i++)
            {
                GL.DeleteBuffer(vbo[i]);
                GL.DeleteBuffer(cbo[i]);
                GL.DeleteBuffer(ebo[i]);
                GL.DeleteVertexArray(vao[i]);

                vbo[i] = cbo[i] = ebo[i] = vao[i] = 0;

                if (meshData[i].vertices != null) meshData[i].vertices = new Vector3[] { new Vector3(0, 0, 0) };
                if (meshData[i].indices != null) meshData[i].indices = new uint[] { 0 };
                if (meshData[i].colors != null) meshData[i].colors = new Vector4[] { new Vector4(0) };

                meshData[i] = default;
            }
            logger.Stop();
            GC.Collect();

            exit = true;
            continueSearch = false;
        }

        public void Run()
        {
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
                    Vsync = VSyncMode.On,
                    WindowBorder = WindowBorder.Resizable
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
            GL.FrontFace(FrontFaceDirection.Cw);

            SetupMatrices();
            shader = new Shader(vertexShaderSource, fragmentShaderSource);

            baseSize = new Vector3(gridSize.X, gridSize.Y, gridSize.Z) * 0.125f + (0.01f, 0.01f, 0.01f);

            _controller = new ImGuiController(window.ClientSize.X, window.ClientSize.Y);

            //usedDirections = mainDirections;

            fileINIT();
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

            //updateGrid();

            if (updateMesh)
            {
                updateMesh = false;

                if (chngDielectricAndConductor)
                {
                    foreach (var node in grid)
                    {
                        if (node.State == NodeState.Dielectric && node.DrawMD == DrawMode.Air)
                        {
                            node.DrawMD = DrawMode.Wall;
                        }
                        if (node.State == NodeState.Conductor && node.DrawMD == DrawMode.Wall)
                        {
                            node.DrawMD = DrawMode.Air;
                        }
                    }
                }
                else
                {
                    foreach (var node in grid)
                    {
                        if (node.State == NodeState.Dielectric && node.DrawMD == DrawMode.Wall)
                        {
                            node.DrawMD = DrawMode.Air;
                        }
                        if (node.State == NodeState.Conductor && node.DrawMD == DrawMode.Air)
                        {
                            node.DrawMD = DrawMode.Wall;
                        }
                    }
                }

                mesher = new(grid, gridSize);
                DisposeMeshData();
                meshData[0] = mesher.GenerateMesh(DrawMode.Wall);
                meshData[1] = mesher.GenerateMesh(DrawMode.Start, DrawMode.End);
                meshData[2] = mesher.GenerateMesh(DrawMode.Path, DrawMode.Open, DrawMode.Closed);
                updateVBOs();
                GC.Collect();
                GC.WaitForPendingFinalizers();
            }
            // update box if necessary //// what box
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

        }

        bool chngDielectricAndConductor = false;
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
            for (int i = 0; i < _bufferCount; i++)
            {
                if (!drawWalls && i == 0)
                {
                    continue;
                }

                GL.BindVertexArray(vao[i]);
                shader.SetVec4("cColor", (0, 0, 0, 0));
                // Draw filled cubes
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
                GL.DrawElements(PrimitiveType.Triangles, meshData[i].indices.Length, DrawElementsType.UnsignedInt, 0);

                // Draw wireframe
                GL.LineWidth(2.0f);
                shader.SetVec4("cColor", (0, 0, 0, 1));
                GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
                GL.DrawElements(PrimitiveType.Triangles, meshData[i].indices.Length, DrawElementsType.UnsignedInt, 0);

                GL.BindVertexArray(0);
            }

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

            useMeshRender();

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
            if (!_isMouseOverMenu)
            {
                // zoom
                zoom -= e.OffsetY * 0.2f;
                zoom = Math.Max(0.1f, Math.Min(zoom, 1000.0f));
                cameraPosition.Z = zoom;
                UpdateViewMatrix();
            }
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
