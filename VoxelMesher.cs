using OpenTK.Mathematics;
using PathFind3D;
using System;

public class VoxelMesher
{
    private readonly GraphNode[,,] grid;
    private readonly Vector3i gridSize;
    private const float CUBE_SCALE = 0.125f; // 1/8 of original size
    private readonly Vector3 offset;

    Vector3[] cubeVertices = new Vector3[]
        {
            new Vector3(-0.5f, -0.5f, -0.5f),
            new Vector3( 0.5f, -0.5f, -0.5f),
            new Vector3( 0.5f,  0.5f, -0.5f),
            new Vector3(-0.5f,  0.5f, -0.5f),
            new Vector3(-0.5f, -0.5f,  0.5f),
            new Vector3( 0.5f, -0.5f,  0.5f),
            new Vector3( 0.5f,  0.5f,  0.5f),
            new Vector3(-0.5f,  0.5f,  0.5f)
        };

    uint[] cubeIndices = new uint[]
    {
            0, 1, 2, 2, 3, 0, // Front
            1, 5, 6, 6, 2, 1, // Right
            5, 4, 7, 7, 6, 5, // Back
            4, 0, 3, 3, 7, 4, // Left
            3, 2, 6, 6, 7, 3, // Top
            4, 5, 1, 1, 0, 4  // Bottom
    };

    public VoxelMesher(GraphNode[,,] grid, Vector3i gridSize)
    {
        this.grid = grid;
        this.gridSize = gridSize;
        this.offset = -(Vector3)gridSize / 2 * CUBE_SCALE;
    }

    public (Vector3[] vertices, uint[] indices, Vector4[] colors) GenerateMesh()
    {
        int maxCubes = gridSize.X * gridSize.Y * gridSize.Z;
        Vector3[] vertices = new Vector3[maxCubes * 8];
        Vector4[] colors = new Vector4[maxCubes * 8];
        uint[] indices = new uint[maxCubes * 36];

        int vertexCount = 0;
        int indexCount = 0;

        for (int x = 0; x < gridSize.X; x++)
        {
            for (int y = 0; y < gridSize.Y; y++)
            {
                for (int z = 0; z < gridSize.Z; z++)
                {
                    if (grid != null && grid[x, y, z].DrawMD == DrawMode.Wall)
                    {
                        AddCube(new Vector3(x, y, z) * CUBE_SCALE + offset, vertices, indices, colors, ref vertexCount, ref indexCount, grid[x, y, z]);
                    }
                }
            }
        }

        // Trim arrays to actual size
        //Array.Resize(ref vertices, vertexCount);
        //Array.Resize(ref colors, vertexCount);
        //Array.Resize(ref indices, indexCount);

        return (vertices, indices, colors);
    }

    private void AddCube(Vector3 GridPosition, Vector3[] vertices, uint[] indices, Vector4[] colors, ref int vertexCount, ref int indexCount, GraphNode node)
    {
        uint startIndex = (uint)vertexCount;
        for (int i = 0; i < 8; i++)
        {
            vertices[vertexCount] = (cubeVertices[i] * CUBE_SCALE) + GridPosition;
            switch (node.DrawMD)
            {
                case DrawMode.Wall:
                    colors[vertexCount] = new Vector4(1, 1, 1, 1);
                    break;

                case DrawMode.Start:
                    colors[vertexCount] = new Vector4(0, 1, 0, 1);
                    break;

                case DrawMode.End:
                    colors[vertexCount] = new Vector4(1, 0, 0, 1);
                    break;
            }
            vertexCount++;
        }

        for (int i = 0; i < cubeIndices.Length; i++)
        {
            indices[indexCount] = startIndex + cubeIndices[i];
            indexCount++;
        }
    }
}