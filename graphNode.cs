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
        //public List<GraphNode> Connections { get; set; } = new();
        public float AspectRatio { get; set; } = 1;
        public Vector3 Rotation { get; set; } = new(0, 0, 0);
        public float BaseSize = 0.125f;
        public DrawMode DrawMD { get; set; } = DrawMode.Wall;

        public float dstToEnd = 0;
        public float dstFromStart = 0;

        private static readonly Vector4 ColorWhite = new Vector4(1, 1, 1, 1);


        int[] cubeIndices =
        {
        0, 1, 2,
        2, 1, 3,
        2, 3, 4,
        4, 3, 5,
        4, 5, 6,
        6, 5, 7,
        6, 7, 0,
        0, 7, 1,
        1, 7, 3,
        3, 7, 5,
        6, 0, 4,
        4, 0, 2
    };

        float[] cubeVertices = new float[8 * 3];

        int[] lineIndices =
        {
        // First Quad
        0, 1, 1, 3, 3, 2, 2, 0,
        // Second Quad
        2, 3, 3, 5, 5, 4, 4, 2,
        // Third Quad
        4, 5, 5, 7, 7, 6, 6, 4,
        // Fourth Quad
        6, 7, 7, 1, 1, 0, 0, 6,
        // Fifth Quad
        1, 7, 7, 5, 5, 3, 3, 1,
        // Sixth Quad
        6, 0, 0, 2, 2, 4, 4, 6
    };

        public GraphNode(Vector3 position)
        {
            Position = position;
            InitializeCubeVertices();
        }

        public GraphNode(float x, float y, float z)
        {
            Position = new Vector3(x, y, z);
            InitializeCubeVertices();
        }

        public GraphNode(Vector3 position, DrawMode drawMode)
        {
            Position = position;
            DrawMD = drawMode;
            InitializeCubeVertices();
        }

        public float DistanceTo(GraphNode other)
        {
            return (other.Position - Position).Length;
        }

        private void InitializeCubeVertices()
        {
            cubeVertices = new float[]
            {
            -0.500000f * BaseSize, -0.500000f * BaseSize, 0.500000f * BaseSize,
            0.500000f * BaseSize, -0.500000f * BaseSize, 0.500000f * BaseSize,
            -0.500000f * BaseSize, 0.500000f * BaseSize, 0.500000f * BaseSize,
            0.500000f * BaseSize, 0.500000f * BaseSize, 0.500000f * BaseSize,
            -0.500000f * BaseSize, 0.500000f * BaseSize, -0.500000f * BaseSize,
            0.500000f * BaseSize, 0.500000f * BaseSize, -0.500000f * BaseSize,
            -0.500000f * BaseSize, -0.500000f * BaseSize, -0.500000f * BaseSize,
            0.500000f * BaseSize, -0.500000f * BaseSize, -0.500000f * BaseSize
            };
        }

        private Matrix4 CreateRotationMatrix(Vector3 rotation)
        {
            Matrix4 rotMatX = Matrix4.CreateRotationX(rotation.X);
            Matrix4 rotMatY = Matrix4.CreateRotationY(rotation.Y);
            Matrix4 rotMatZ = Matrix4.CreateRotationZ(rotation.Z);
            return rotMatZ * rotMatY * rotMatX;
        }

        private void drawWireframe()
        {
            // Draw wireframe overlay
            GL.Color4(0.0f, 0.0f, 0.0f, 1.0f);
            GL.PolygonMode(MaterialFace.Front, PolygonMode.Line);
            GL.VertexPointer(3, VertexPointerType.Float, 0, cubeVertices);
            GL.DrawElements(PrimitiveType.Lines, lineIndices.Length, DrawElementsType.UnsignedInt, lineIndices);
        }

        private void fillCube(Vector4 color)
        {
            // Draw filled polygons
            GL.Color4(color);
            GL.PolygonMode(MaterialFace.Front, PolygonMode.Fill);
            GL.VertexPointer(3, VertexPointerType.Float, 0, cubeVertices);
            GL.DrawElements(PrimitiveType.Triangles, cubeIndices.Length, DrawElementsType.UnsignedInt, cubeIndices);
        }

        public void Draw(Matrix4 viewMatrix, Matrix4 projectionMatrix)
        {
            if (DrawMD == DrawMode.Air)
                return;

            // Ensure cube vertices are initialized once
            if (cubeVertices[0] != -0.500000f * BaseSize)
            {
                InitializeCubeVertices();
            }

            Matrix4 rotationMatrix = CreateRotationMatrix(Rotation);
            Matrix4 modelMatrix = Matrix4.CreateTranslation(Position) * rotationMatrix;
            Matrix4 mvpMatrix = modelMatrix * viewMatrix * projectionMatrix;

            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref mvpMatrix);

            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.FrontFace(FrontFaceDirection.Ccw);

            GL.EnableClientState(ArrayCap.VertexArray);

            switch (DrawMD)
            {

                case DrawMode.Wireframe:
                    drawWireframe();
                    break;

                case DrawMode.Wall:
                    fillCube(ColorWhite);
                    drawWireframe();
                    break;

                case DrawMode.Start:
                    fillCube((0, 1, 0, 1));
                    drawWireframe();
                    break;

                case DrawMode.End:
                    fillCube((1, 0, 0, 1));
                    drawWireframe();
                    break;

                case DrawMode.Open:
                    fillCube((1, 0.8470588235f, 0, 0.1f));
                    drawWireframe();
                    break;

                case DrawMode.Path:
                    fillCube((0, 1, 1, 0.1f));
                    drawWireframe();
                    break;

                case DrawMode.Closed:
                    // fillCube(new Vector4(1f, 1f, 0.1f, 0.2f));
                    break;
            }

            GL.DisableClientState(ArrayCap.VertexArray);
            GL.Disable(EnableCap.CullFace);
        }
    }
}
