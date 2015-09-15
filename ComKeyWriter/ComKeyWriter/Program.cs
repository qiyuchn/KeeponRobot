    using System;
using System.IO.Ports;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ComKeyWriter
{
    class Program
    {
        static void Main(string[] args)
        {
            using (var port = new SerialPort("COM3", 9600))
            {
                port.Open();
                while (true)
                {
                    var key = Console.ReadKey();
                    var keyChar = key.KeyChar.ToString().ToUpper();
                    if (key.Key == ConsoleKey.Escape)
                    {
                        break;
                    }
                    else
                    {
                        port.Write(keyChar);
                        Console.WriteLine("Entered: " + keyChar);
                    }

                }
                port.Close();
            }
        }
    }
}
