using UnityEngine;
using System.IO;
using System.Linq;

public class DataLogger : MonoBehaviour
{
    CsvLogger csvLogger;
    void Start()
    {
        csvLogger = new CsvLogger("example.csv", "positionX", "positionY", "time");
    }

    // Update is called once per frame
    void Update()
    {
        csvLogger.AddRow(transform.position.x, transform.position.y, Time.time);
    }

    private void OnApplicationQuit() {
        csvLogger.Close();
    }
}

public class CsvLogger
{
    StreamWriter file;
    public CsvLogger(string filename)
    {
        file = new StreamWriter(filename);
    }

    public CsvLogger(string filename, params string[] columnNames): this(filename)
    {
        file.WriteLine(string.Join(",", columnNames));
    }

    public void AddRow(params float[] values)
    {
        file.WriteLine(string.Join(",", values.Select(f=>f.ToString())));
    }

    public void Close()
    {
        file.Close();
    }

}
