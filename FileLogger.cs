using System;
using System.Collections.Concurrent;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

public class FileLogger
{
    private readonly string _logFilePath;
    private readonly ConcurrentQueue<string> _logQueue = new ConcurrentQueue<string>();
    private readonly CancellationTokenSource _cts = new CancellationTokenSource();
    private Task _logWriterTask = new Task(() => { });

    public FileLogger(string logFilePath)
    {
        _logFilePath = logFilePath;
        StartLogWriter();
    }

    public void WriteLine(string value)
    {
        _logQueue.Enqueue(value + Environment.NewLine);
        Console.WriteLine(value);
    }
    public void WriteLine(int value)
    {
        _logQueue.Enqueue(value.ToString() + Environment.NewLine);
        Console.WriteLine(value);
    }
    public void WriteLine(double value)
    {
        _logQueue.Enqueue(value.ToString() + Environment.NewLine);
        Console.WriteLine(value);
    }
    public void WriteLine(bool value)
    {
        _logQueue.Enqueue(value.ToString() + Environment.NewLine);
        Console.WriteLine(value);
    }
    public void WriteLine(float value)
    {
        _logQueue.Enqueue(value.ToString() + Environment.NewLine);
        Console.WriteLine(value);
    }
    public void WriteLine(char value)
    {
        _logQueue.Enqueue(value.ToString() + Environment.NewLine);
        Console.WriteLine(value);
    }

    private void StartLogWriter()
    {
        _logWriterTask = Task.Run(async () =>
        {
            while (!_cts.Token.IsCancellationRequested)
            {
                try
                {
                    while (_logQueue.TryDequeue(out string? logMessage))
                    {
                        await WriteToFileAsync(logMessage);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Error writing to log file: {ex.Message}");
                }

                await Task.Delay(10);
            }
        }, _cts.Token);
    }

    private async Task WriteToFileAsync(string text)
    {
        try
        {
            using (var writer = new StreamWriter(_logFilePath, append: true))
            {
                await writer.WriteLineAsync(text);
            }
        }
        catch (IOException)
        {
            _logQueue.Enqueue(text);
        }
    }

    public void Stop()
    {
        _cts.Cancel();
        _logWriterTask.Wait();
    }
}
