using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PathFind3D
{
    public class VoxelMesher
    {
        private readonly GraphNode[,,] grid;
        private readonly Vector3i gridSize;

        public VoxelMesher(GraphNode[,,] grid, Vector3i gridSize)
        {
            this.grid = grid;
            this.gridSize = gridSize;
        }

        public (List<Vector3> vertices, List<int> indices, List<Vector3> normals) GenerateMesh()
        {
            var vertices = new List<Vector3>();
            var indices = new List<int>();
            var normals = new List<Vector3>();

            if (grid == null)
            {
                return (vertices, indices, normals);
            }

            for (int x = 0; x < gridSize.X; x++)
            {
                for (int y = 0; y < gridSize.Y; y++)
                {
                    for (int z = 0; z < gridSize.Z; z++)
                    {
                        if (grid[x, y, z] != null && grid[x, y, z].DrawMD == DrawMode.Wall)
                        {
                            AddCubeFaces(new Vector3i((x - grid.GetLength(0) / 2), (y - grid.GetLength(1) / 2), (z - grid.GetLength(2) / 2)), vertices, indices, normals);
                        }
                    }
                }
            }

            return (vertices, indices, normals);
        }

        private void AddCubeFaces(Vector3i position, List<Vector3> vertices, List<int> indices, List<Vector3> normals)
        {
            int[][] faceIndices = new int[][]
            {
            new int[] { 0, 1, 2, 3 }, // Front
            new int[] { 1, 5, 6, 2 }, // Right
            new int[] { 5, 4, 7, 6 }, // Back
            new int[] { 4, 0, 3, 7 }, // Left
            new int[] { 3, 2, 6, 7 }, // Top
            new int[] { 4, 5, 1, 0 }  // Bottom
            };

            Vector3[] faceNormals = new Vector3[]
            {
            new Vector3(0, 0, 1),
            new Vector3(1, 0, 0),
            new Vector3(0, 0, -1),
            new Vector3(-1, 0, 0),
            new Vector3(0, 1, 0),
            new Vector3(0, -1, 0)
            };

            for (int face = 0; face < 6; face++)
            {
                if (ShouldRenderFace(position, face))
                {
                    int vIndex = vertices.Count;
                    for (int i = 0; i < 4; i++)
                    {
                        vertices.Add(GetVertexPosition(position, faceIndices[face][i]));
                        normals.Add(faceNormals[face]);
                    }

                    indices.AddRange(new int[] { vIndex, vIndex + 1, vIndex + 2, vIndex + 2, vIndex + 1, vIndex + 3 });
                }
            }
        }

        private bool ShouldRenderFace(Vector3i position, int face)
        {
            Vector3i checkPos = position;
            switch (face)
            {
                case 0: checkPos.Z += 1; break; // Front
                case 1: checkPos.X += 1; break; // Right
                case 2: checkPos.Z -= 1; break; // Back
                case 3: checkPos.X -= 1; break; // Left
                case 4: checkPos.Y += 1; break; // Top
                case 5: checkPos.Y -= 1; break; // Bottom
            }

            return !IsInGrid(checkPos) || grid[checkPos.X, checkPos.Y, checkPos.Z].DrawMD == DrawMode.Air;
        }

        private bool IsInGrid(Vector3i pos)
        {
            return pos.X >= 0 && pos.X < gridSize.X &&
                   pos.Y >= 0 && pos.Y < gridSize.Y &&
                   pos.Z >= 0 && pos.Z < gridSize.Z;
        }

        private Vector3 GetVertexPosition(Vector3i gridPos, int vertexIndex)
        {
            float x = gridPos.X + (vertexIndex & 1);
            float y = gridPos.Y + ((vertexIndex & 2) >> 1);
            float z = gridPos.Z + ((vertexIndex & 4) >> 2);
            return new Vector3(x / 8, y / 8, z / 8);
        }
    }
}
