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

        public void Draw(Matrix4 viewMatrix, Matrix4 projectionMatrix, Shader shader, int indexCount, int vao)
        {
            if (DrawMD == DrawMode.Air)
                return;

            // Create model matrix using position and rotation for this node
            Matrix4 rotationMatrix = CreateRotationMatrix(Rotation);
            Matrix4 modelMatrix = Matrix4.CreateTranslation(Position) * rotationMatrix;

            // Activate shader and set transformation matrices
            shader.Use();
            shader.SetMatrix4("model", modelMatrix);
            shader.SetMatrix4("view", viewMatrix);
            shader.SetMatrix4("projection", projectionMatrix);

            // Enable OpenGL states
            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.FrontFace(FrontFaceDirection.Ccw);

            GL.BindVertexArray(vao);  // Use the VAO passed as a parameter

            // Render based on the current Draw Mode (DrawMD)
            switch (DrawMD)
            {
                case DrawMode.Wireframe:
                    RenderWireframe(shader, new Vector4(0, 0, 0, 1), indexCount);
                    break;

                case DrawMode.Wall:
                    RenderFilled(shader, ColorWhite, indexCount);
                    RenderWireframe(shader, new Vector4(0, 0, 0, 1), indexCount);
                    break;

                case DrawMode.Start:
                    RenderFilled(shader, new Vector4(0, 1, 0, 1), indexCount);
                    RenderWireframe(shader, new Vector4(0, 0, 0, 1), indexCount);
                    break;

                case DrawMode.End:
                    RenderFilled(shader, new Vector4(1, 0, 0, 1), indexCount);
                    RenderWireframe(shader, new Vector4(0, 0, 0, 1), indexCount);
                    break;

                case DrawMode.Open:
                    RenderFilled(shader, new Vector4(1, 0.847f, 0, 0.1f), indexCount);
                    RenderWireframe(shader, new Vector4(0, 0, 0, 1), indexCount);
                    break;

                case DrawMode.Path:
                    RenderFilled(shader, new Vector4(0, 1, 1, 0.25f), indexCount);
                    RenderWireframe(shader, new Vector4(0, 0, 0, 1), indexCount);
                    break;

                case DrawMode.Closed:
                    // Uncomment if needed
                    // RenderFilled(shader, new Vector4(1f, 1f, 0.1f, 0.2f), indexCount);
                    break;
            }

            GL.BindVertexArray(0);

            GL.Disable(EnableCap.DepthTest);
            GL.Disable(EnableCap.CullFace);
        }

        // Helper method for rendering filled shapes
        private void RenderFilled(Shader shader, Vector4 color, int indexCount)
        {
            shader.SetVec4("cColor", color);
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Fill);
            GL.DrawElements(PrimitiveType.Triangles, indexCount, DrawElementsType.UnsignedInt, IntPtr.Zero);
        }

        // Helper method for rendering wireframe shapes
        private void RenderWireframe(Shader shader, Vector4 color, int indexCount)
        {
            shader.SetVec4("cColor", color);
            GL.PolygonMode(MaterialFace.FrontAndBack, PolygonMode.Line);
            GL.LineWidth(2.0f);
            GL.DrawElements(PrimitiveType.Triangles, indexCount, DrawElementsType.UnsignedInt, IntPtr.Zero);
        }

    }
}
