using System;
using System.IO;
using System.Net;
using System.Text;
using System.Net.Sockets;
using System.Globalization;
using System.Linq.Expressions;

class Program
{
    const int PORT_NO = 65433;
    const string SERVER_IP = "127.0.0.1";
    static string textToSend;
    static byte[] bytesToSend;
    static byte[] bytesToRead;
    static int bytesRead;
    static void Main(string[] args)
    {
        //---create a TCPClient object at the IP and port no.---
        TcpClient client;

        while (true)
        {

            while (true)
            {
                try
                {
                    client = new TcpClient(SERVER_IP, PORT_NO);
                }
                catch (SocketException e)
                {
                    Console.WriteLine("ERROR: can't connect to Python script!");
                    continue;
                }
                Console.WriteLine("Connected to Python script!");
                break;
            }
            NetworkStream nwStream = client.GetStream();

            while (true)
            {

                //---read back the text---
                try
                {
                    bytesToRead = new byte[client.ReceiveBufferSize];
                    bytesRead = nwStream.Read(bytesToRead, 0, client.ReceiveBufferSize);
                }catch (System.IO.IOException e)
                {
                    Console.WriteLine("ERROR: can't read from socket");
                    break;
                }
                string receivedString = Encoding.ASCII.GetString(bytesToRead, 0, bytesRead);
                List<string> result = receivedString.Remove(receivedString.Length - 1, 1).Remove(0, 1).Split(',').ToList();


                if (result[0] == "THERMOPILE_COG")
                {
                    float value = (float)Convert.ToDouble(result[1]); // difference of two temperature values

                    // this value is the difference between a temple measurement and the tip of the nose
                    //   (i.e., TEMPLE_TEMP - NOSE_TEMP = THERMOPILE_COG)
                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "THERMOPILE_TEMPLE")
                {
                    // this value is temperature, in Kelvin, of the middle thermopile of the temple thermopile array of 3
                    float value = (float)Convert.ToDouble(result[1]); // kelvin

                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "SGP_GAS")
                {
                    // reference datasheet for SGP41 on how the indices are defined
                    float voc_index = (float)Convert.ToDouble(result[1]); // range: 1-500, ambient average: 100
                    float nox_index = (float)Convert.ToDouble(result[2]); // range: 1-500, ambient average: 1

                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "BME_IAQ")
                {
                    /**
                        * @brief Indoor-air-quality estimate [0-500]
                        * 
                        * Indoor-air-quality (IAQ) gives an indication of the relative change in ambient TVOCs detected by BME680. 
                        * 
                        * @note The IAQ scale ranges from 0 (clean air) to 500 (heavily polluted air). During operation, algorithms 
                        * automatically calibrate and adapt themselves to the typical environments where the sensor is operated 
                        * (e.g., home, workplace, inside a car, etc.).This automatic background calibration ensures that users experience 
                        * consistent IAQ performance. The calibration process considers the recent measurement history (typ. up to four 
                        * days) to ensure that IAQ=25 corresponds to typical good air and IAQ=250 indicates typical polluted air.
                        */
                    float IAQ = (float)Convert.ToDouble(result[1]);

                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "BME_CO2_EQ")
                {
                    float co2_eq = (float)Convert.ToDouble(result[1]); // co2 equivalent estimate [ppm]

                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "BME_VOC_EQ")
                {
                    float voc_index = (float)Convert.ToDouble(result[1]); // breath VOC concentration estimate [ppm]
                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "SHT45")
                {
                    float ambient_temperature = (float)Convert.ToDouble(result[1]); // Celsius
                    float ambient_humidity = (float)Convert.ToDouble(result[2]); // relative humidity, influenced by person breathing

                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "Lux")
                {
                    float lux = (float)Convert.ToDouble(result[1]); // light intensity [lux]

                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "Blink")
                {
                    // !!!!! NOT IMPLEMENTED !!!!! 
                    float blink = (float)Convert.ToDouble(result[1]); // blink: 1, no_blink: 0

                    Console.WriteLine(result[0]);
                }
                else if (result[0] == "Spec")
                {
                    float red = (float)Convert.ToDouble(result[1]); // 0->1.0
	            float green = (float)Convert.ToDouble(result[2]); // 0->1.0
		    float blue = (float)Convert.ToDouble(result[3]); // 0->1.0

                    Console.WriteLine(result[0]);
                }
            }
            
        }
        //client.Close();
    }
}