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
        public bool canGenerate = true;
        public NodeState State { get; set; } = NodeState.Dielectric;
        public DrawMode DrawMD { get; set; } = DrawMode.Wall;
        public float gScore { get; set; } = float.MaxValue;
        public float hScore { get; set; } = 0;
        public float fScore { get; set; } = float.MaxValue;
        public GraphNode(Vector3 position)
        {
            Position = position;
        }
        public GraphNode()
        {

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
    }
}
