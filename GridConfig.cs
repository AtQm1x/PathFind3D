using OpenTK.Mathematics;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace PathFind3D
{
    using System;
    using System.IO;
    using System.Threading;

    public class GridConfig
    {
        public Vector3i GridSize { get; set; }
        public float NodeDensity { get; set; }
        public Vector3i StartNodePos { get; set; }
        public Vector3i EndNodePos { get; set; }

        public void ReadConfigFromFile(string filePath)
        {
            EnsureConfigFileExists(filePath);

            foreach (string line in File.ReadLines(filePath))
            {
                string trimmedLine = line.Trim();
                if (string.IsNullOrWhiteSpace(trimmedLine) || trimmedLine.StartsWith("#"))
                    continue;

                string[] parts = trimmedLine.Split(new[] { '=' }, 2);
                if (parts.Length != 2)
                    throw new FormatException($"Invalid line format: {line}");

                string key = parts[0].Trim().ToLowerInvariant();
                string value = parts[1].Trim();

                switch (key)
                {
                    case "gridsize":
                        GridSize = ParseVector3i(value);
                        break;
                    case "nodedensity":
                        NodeDensity = ParseFloat(value);
                        break;
                    case "startnodepos":
                        StartNodePos = ParseVector3i(value);
                        break;
                    case "endnodepos":
                        EndNodePos = ParseVector3i(value);
                        break;
                    default:
                        throw new FormatException($"Unknown configuration key: {key}");
                }
            }

            ValidateConfig();
        }

        private void EnsureConfigFileExists(string filePath)
        {
            if (File.Exists(filePath)) return;

            string[] defaultConfig = {
            "gridSize = (32, 32, 32)",
            "nodeDensity = 0,1",
            "startNodePos = (0, 0, 0)",
            "endNodePos = (31, 31, 31)"
        };

            File.WriteAllLines(filePath, defaultConfig);
            Thread.Sleep(100); // Ensure file is fully written before reading
        }

        private Vector3i ParseVector3i(string value)
        {
            value = value.Trim('(', ')');
            string[] parts = value.Split(',');
            if (parts.Length != 3)
                throw new FormatException($"Invalid Vector3i format: {value}");

            return new Vector3i(
                ParseInt(parts[0]),
                ParseInt(parts[1]),
                ParseInt(parts[2])
            );
        }

        private int ParseInt(string value)
        {
            if (!int.TryParse(value.Trim(), out int result))
                throw new FormatException($"Invalid integer format: {value}");
            return result;
        }

        private float ParseFloat(string value)
        {
            if (!float.TryParse(value.Trim(), out float result))
            {
                if (!float.TryParse(value.Replace(',', '.').Trim(), out float result1))
                    throw new FormatException($"Invalid float format: {value}");
                return result1;
            }
            return result;
        }

        private void ValidateConfig()
        {
            if (GridSize.X <= 0 || GridSize.Y <= 0 || GridSize.Z <= 0)
                throw new InvalidOperationException("GridSize must be positive in all dimensions");

            if (NodeDensity <= 0 || NodeDensity > 1)
                throw new InvalidOperationException("NodeDensity must be between 0 and 1");

            if (!IsValidPosition(StartNodePos) || !IsValidPosition(EndNodePos))
                throw new InvalidOperationException("Start and end positions must be within the grid");
        }

        private bool IsValidPosition(Vector3i pos)
        {
            return pos.X >= 0 && pos.X <= GridSize.X &&
                   pos.Y >= 0 && pos.Y <= GridSize.Y &&
                   pos.Z >= 0 && pos.Z <= GridSize.Z;
        }
    }
}
