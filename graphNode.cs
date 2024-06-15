using OpenTK.Mathematics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OpenTKBase
{
    internal class GraphNode
    {
        public Vector3 Position { get; private set; }
        public GraphNode? Parent { get; set; } = null;
        public List<GraphNode> Connections { get; } = new();
        public float AspectRatio { get; set; } = 1;
        public Vector3 Rotation { get; set; } = new(0, 0, 0);
        public float BaseSize = 0.125f;
        public DrawMode DrawMD { get; set; } = DrawMode.Both;
        public enum DrawMode { Fill, Wireframe, Both, None };
        public Vector3 center = new(0, 0, 0);

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
                // Front face
                0, 1, 1, 2, 2, 3, 3, 0,
                // Back face
                4, 5, 5, 6, 6, 7, 7, 4,
                // Connecting edges
                0, 4, 1, 5, 2, 6, 3, 7
            };

        public GraphNode(Vector3 position)
        {
            Position = position;
            updateCenter(Matrix4.Identity, Matrix4.Identity);
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

        public GraphNode(float x, float y, float z)
        {
            Position = (x, y, z);
            updateCenter(Matrix4.Identity, Matrix4.Identity);
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

        public GraphNode(Vector3 position, DrawMode drawMode)
        {
            Position = position;
            updateCenter(Matrix4.Identity, Matrix4.Identity);
            DrawMD = drawMode;
        }

        private Matrix4 CreateRotationMatrix(Vector3 rotation)
        {
            Matrix4 rotMatX = Matrix4.CreateRotationX(rotation.X);
            Matrix4 rotMatY = Matrix4.CreateRotationY(rotation.Y);
            Matrix4 rotMatZ = Matrix4.CreateRotationZ(rotation.Z);
            return rotMatZ * rotMatY * rotMatX;
        }

        public void updateCenter(Matrix4 viewMatrix, Matrix4 projectionMatrix)
        {
            center = Vector3.TransformPosition(Position, CreateRotationMatrix(Rotation) * viewMatrix * projectionMatrix);
        }

        public void Draw(Matrix4 viewMatrix, Matrix4 projectionMatrix)
        {
            if (cubeVertices[0] != -0.500000f * BaseSize)
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

            Matrix4 rotationMatrix = CreateRotationMatrix(Rotation);
            Matrix4 modelMatrix = Matrix4.CreateTranslation(Position) * rotationMatrix;
            Matrix4 mvpMatrix = modelMatrix * viewMatrix * projectionMatrix;

            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref mvpMatrix);

            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.FrontFace(FrontFaceDirection.Ccw);

            GL.EnableClientState(ArrayCap.VertexArray);

            if (DrawMD == DrawMode.Fill || DrawMD == DrawMode.Both)
            {
                // Draw filled polygons
                GL.Color4(1.0f, 1.0f, 1.0f, 1.0f);
                GL.PolygonMode(MaterialFace.Front, PolygonMode.Fill);
                GL.VertexPointer(3, VertexPointerType.Float, 0, cubeVertices);
                GL.DrawElements(PrimitiveType.Triangles, cubeIndices.Length, DrawElementsType.UnsignedInt, cubeIndices);
            }

            if (DrawMD == DrawMode.Wireframe || DrawMD == DrawMode.Both)
            {
                // Draw wireframe overlay
                GL.Color4(0.0f, 0.0f, 0.0f, 1.0f);
                GL.PolygonMode(MaterialFace.Front, PolygonMode.Line);
                GL.VertexPointer(2, VertexPointerType.Float, 0, cubeVertices);
                GL.DrawElements(PrimitiveType.Lines, lineIndices.Length, DrawElementsType.UnsignedInt, lineIndices);
            }

            GL.DisableClientState(ArrayCap.VertexArray);
            GL.Disable(EnableCap.CullFace);
        }

    }

}
