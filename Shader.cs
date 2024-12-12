using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using System;

namespace PathFind3D
{
    public class Shader
    {
        public readonly int Handle;

        public Shader(string vertexShaderSource, string fragmentShaderSource)
        {
            int vertexShader = CompileShader(ShaderType.VertexShader, vertexShaderSource);
            int fragmentShader = CompileShader(ShaderType.FragmentShader, fragmentShaderSource);

            Handle = GL.CreateProgram();

            GL.AttachShader(Handle, vertexShader);
            GL.AttachShader(Handle, fragmentShader);

            LinkProgram(Handle);

            GL.DetachShader(Handle, vertexShader);
            GL.DetachShader(Handle, fragmentShader);
            GL.DeleteShader(vertexShader);
            GL.DeleteShader(fragmentShader);
        }

        private static int CompileShader(ShaderType type, string source)
        {
            int shader = GL.CreateShader(type);
            GL.ShaderSource(shader, source);
            GL.CompileShader(shader);

            GL.GetShader(shader, ShaderParameter.CompileStatus, out int success);
            if (success == 0)
            {
                string infoLog = GL.GetShaderInfoLog(shader);
                Console.WriteLine($"error compiling shader of type {type}: {infoLog}");
            }

            return shader;
        }

        private static void LinkProgram(int program)
        {
            GL.LinkProgram(program);

            GL.GetProgram(program, GetProgramParameterName.LinkStatus, out int success);
            if (success == 0)
            {
                string infoLog = GL.GetProgramInfoLog(program);
                Console.WriteLine($"shader linker error: {infoLog}");
            }
        }

        public void Use()
        {
            GL.UseProgram(Handle);
        }

        public void SetMatrix4(string name, Matrix4 matrix)
        {
            int location = GL.GetUniformLocation(Handle, name);
            GL.UniformMatrix4(location, false, ref matrix);
        }

        public void Uniform1(string name, int val)
        {
            int location = GL.GetUniformLocation(Handle, name);
            GL.Uniform1(location, val);
        }
        public void Uniform1(string name, float val)
        {
            int location = GL.GetUniformLocation(Handle, name);
            GL.Uniform1(location, val);
        }
        public void Uniform1(string name, double val)
        {
            int location = GL.GetUniformLocation(Handle, name);
            GL.Uniform1(location, val);
        }
        public void Uniform1(string name, uint val)
        {
            int location = GL.GetUniformLocation(Handle, name);
            GL.Uniform1(location, val);
        }

        public void SetVec2(string name, Vector2 vec)
        {
            int location = GL.GetUniformLocation(Handle, name);
            GL.Uniform2(location, vec.X, vec.Y);
        }

        public void SetVec3(string name, Vector3 vec)
        {
            int location = GL.GetUniformLocation(Handle, name);
            GL.Uniform3(location, vec.X, vec.Y, vec.Z);
        }

        public void SetVec4(string name, Vector4 vec)
        {
            int location = GL.GetUniformLocation(Handle, name);
            GL.Uniform4(location, vec.X, vec.Y, vec.Z, vec.W);
        }
    }
}
