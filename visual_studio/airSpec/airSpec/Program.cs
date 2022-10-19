using System;
using System.IO;
using System.Net;
using System.Text;
using System.Net.Sockets;
class Program
{
    const int PORT_NO = 65432;
    const string SERVER_IP = "127.0.0.1";
    static string textToSend;
    static byte[] bytesToSend;
    static byte[] bytesToRead;
    static int bytesRead;
    static void Main(string[] args)
    {
        //---create a TCPClient object at the IP and port no.---
        TcpClient client = new TcpClient(SERVER_IP, PORT_NO);
        NetworkStream nwStream = client.GetStream();

        while (true)
        {

            //---read back the text---
            bytesToRead = new byte[client.ReceiveBufferSize];
            bytesRead = nwStream.Read(bytesToRead, 0, client.ReceiveBufferSize);
            string receivedString = Encoding.ASCII.GetString(bytesToRead, 0, bytesRead);
            List<string> result = receivedString.Remove(receivedString.Length-1,1).Remove(0, 1).Split(',').ToList();
            //for (int i = 0; i < result.Count; i++)
            //{
             //   Console.WriteLine(result[i]);
            //}
            Console.WriteLine(result[0]);

            //Console.WriteLine("Received : " + Encoding.ASCII.GetString(bytesToRead, 0, bytesRead));
        }
        client.Close();
    }
}