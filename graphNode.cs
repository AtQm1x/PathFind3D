using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using System;
using System.Collections.Generic;

namespace PathFind3D
{
    public class GraphNode
    {
        public Vector3 Position { get; set; }
        public Vector3i GridPosition { get; set; }
        public GraphNode? Parent { get; set; } = null;
        public float AspectRatio { get; set; } = 1;
        public Vector3 Rotation { get; set; } = new Vector3(0, 0, 0);
        public Vector3 BaseSize = new Vector3(0.125f);

        public bool canGenerate = true;
        public NodeState State { get; set; } = NodeState.Dielectric;
        public DrawMode DrawMD { get; set; } = DrawMode.Wall;

        public float dstFromStart = float.MaxValue;
        public float gScore { get; set; } = float.MaxValue;
        public float hScore { get; set; } = 0;
        public float fScore { get; set; } = float.MaxValue;

        private static readonly Vector4 ColorWhite = new Vector4(1, 1, 1, 1);

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

        public GraphNode(Vector3 position)
        {
            Position = position;
        }
        public GraphNode(float x, float y, float z)
        {
            Position = new Vector3(x, y, z);
        }

        public GraphNode(Vector3i GridPosition)
        {
            this.GridPosition = GridPosition;
            this.Position = (Vector3)GridPosition * 0.125f;
        }

        public GraphNode(Vector3i GridPosition, DrawMode drawMode)
        {
            this.GridPosition = GridPosition;
            DrawMD = drawMode;
            this.Position = (Vector3)GridPosition * 0.125f;
        }

        public float DistanceTo(GraphNode other)
        {
            return (other.Position - Position).Length;
        }

        private Matrix4 CreateRotationMatrix(Vector3 rotation)
        {
            Matrix4 rotMatX = Matrix4.CreateRotationX(rotation.X);
            Matrix4 rotMatY = Matrix4.CreateRotationY(rotation.Y);
            Matrix4 rotMatZ = Matrix4.CreateRotationZ(rotation.Z);
            return rotMatZ * rotMatY * rotMatX;
        }
    }
}
