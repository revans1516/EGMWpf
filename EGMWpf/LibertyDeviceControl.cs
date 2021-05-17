//  VSS $Header: /PiDevTools11/Inc/PDIg4.h 18    1/09/14 1:05p Suzanne $  
using System.Collections;
using System.Net.Sockets;
using System;
using System.Text;
using System.Net;
using System.Threading;
using System.IO;
using System.Numerics;

namespace EGMWpf
{
    public enum PlTracker
    {
        Liberty,
        Patriot,
        G4,
        Fastrak,
    };

    public class PlStream
    {
        // port used for our UDP connection
        public int port = 5123;

        // tracker descriptors
        public PlTracker tracker_type = PlTracker.G4;
        public int max_systems = 1;
        public int max_sensors = 3;

        // slots used to store tracker output data
        public bool[] active;
        public uint[] digio;
        public Vector3[] positions;
        public Vector4[] orientations;

        // internal state
        private int max_slots;
        private UdpClient udpClient;
        private Thread conThread;
        private bool stopListening;

        // Use this for initialization
        public PlStream (PlTracker trackerType)
        {
            try
            {
                // there are some constraints between tracking systems
                switch (trackerType)
                {
                    case PlTracker.Liberty:
                        // liberty is a single tracker system
                        max_systems = (max_systems > 1) ? 1 : max_systems;
                        max_sensors = (max_sensors > 16) ? 16 : max_sensors;
                        break;
                    case PlTracker.Patriot:
                        max_systems = (max_systems > 1) ? 1 : max_systems;
                        max_sensors = (max_sensors > 2) ? 2 : max_sensors;
                        break;
                    case PlTracker.G4:
                        // all G4 hubs (systems) have a maximum of 3 sensors
                        max_sensors = (max_sensors > 3) ? 3 : max_sensors;
                        break;
                    case PlTracker.Fastrak:
                        max_systems = (max_systems > 1) ? 1 : max_systems;
                        max_sensors = (max_sensors > 4) ? 4 : max_sensors;
                        break;
                    default:
                        throw new Exception("[polhemus] Unknown Tracker selected in PlStream::Awake().");
                }

                // set the number of slots
                max_slots = max_sensors * max_systems;

                // allocate resources for those slots
                active = new bool[max_slots];
                digio = new uint[max_slots];
                positions = new Vector3[max_slots];
                orientations = new Vector4[max_slots];

                // initialize the slots
                for (int i = 0; i < max_slots; ++i)
                {
                    active[i] = false;
                    digio[i] = 0;
                    positions[i] = new Vector3();
                    orientations[i] = new Vector4();
                }

                switch (trackerType)
                {
                    case PlTracker.Liberty:
                    case PlTracker.Patriot:
                        conThread = new Thread(new ThreadStart(read_liberty));
                        break;
                    case PlTracker.G4:
                        conThread = new Thread(new ThreadStart(read_g4));
                        break;
                    case PlTracker.Fastrak:
                        conThread = new Thread(new ThreadStart(read_fastrak));
                        break;
                    default:
                        throw new Exception("[polhemus] Unknown Tracker selected in PlStream::Awake().");
                }

                // start the read thread
                
            }
            catch (Exception e)
            {
                
                Console.WriteLine("[polhemus] PlStream terminated in PlStream::Awake().");
            }
        }

        // Update is called once per frame
        public void StartReading()
        {
            conThread.Start();
        }

        // read thread
        private void read_g4()
        {
            stopListening = false;

            udpClient = new UdpClient(port);
            IPEndPoint groupEP = new IPEndPoint(IPAddress.Any, port);

            try
            {
                // create temp_active to mark slots
                bool[] temp_active = new bool[max_slots];

                // using G4 frame configuration
                while (!stopListening)
                {
                    byte[] receiveBytes = udpClient.Receive(ref groupEP);

                    //Debug.LogFormat("receiveBytes {0}", new object[]{ receiveBytes.Length });

                    // set slots to inactive
                    for (var i = 0; i < max_slots; ++i)
                        temp_active[i] = false;

                    // offset into buffer
                    int offset = 0;

                    // process body (32*3 = 96 bytes)
                    while (offset + 112 <= receiveBytes.Length)
                    {
                        //Debug.LogFormat("offset {0}", new object[] { offset });

                        // process header (16 bytes)
                        uint nHubID = BitConverter.ToUInt32(receiveBytes, offset + 0);
                        //int nFrame = BitConverter.ToInt32(receiveBytes, offset + 4);
                        uint dwSMap = BitConverter.ToUInt32(receiveBytes, offset + 8);
                        uint dwDgIO = BitConverter.ToUInt32(receiveBytes, offset + 12);

                        //Debug.LogFormat("nHubID = {0}, dwSMap = 0x{1:X}, dwDgIO = 0x{2:X}", new object[] { nHubID, dwSMap, dwDgIO });

                        // adjust offset
                        offset += 16;

                        

                        for (int i = 0, mask = 0x01; i < 3; ++i, mask <<= 1)
                        {
                        
                            // only read sensor results when valid
                            if ((dwSMap & mask) != 0)
                            {
                                // process sensor (32 bytes)
                                int nSenID = BitConverter.ToInt32(receiveBytes, offset);

                                // hub nums are 0-based.
                                uint slot = (nHubID) * 3 + (uint)nSenID;

                                //Debug.LogFormat("nSenID = {0}, slot = nHub*3+nSen = 0x{1:X}, max_slots = 0x{2:X}", new object[] { nSenID, slot, max_slots });

                                // test that we can actually store these results
                                if (slot > max_slots)
                                    throw new Exception("[polhemus] HubID * 3 + SenID is greater than max_slots in PlStream::read().");

                                // here we have the positions
                                float t = BitConverter.ToSingle(receiveBytes, offset + 4);
                                float u = BitConverter.ToSingle(receiveBytes, offset + 8);
                                float v = BitConverter.ToSingle(receiveBytes, offset + 12);

                                // and quaternions
                                float w = BitConverter.ToSingle(receiveBytes, offset + 16);
                                float x = BitConverter.ToSingle(receiveBytes, offset + 20);
                                float y = BitConverter.ToSingle(receiveBytes, offset + 24);
                                float z = BitConverter.ToSingle(receiveBytes, offset + 28);

                                // store results
                                var index = (nHubID) * 3 + nSenID;
                                temp_active[index] = true;
                                digio[index] = dwDgIO;
                                positions[index] = new Vector3(t, u, v);
                                orientations[index] = new Vector4(w, x, y, z);
                            }

                            // always adjust offset
                            offset += 32;
                        }
                    }

                    // mark active slots
                    for (var i = 0; i < max_slots; ++i)
                        active[i] = temp_active[i];
                }
            }
            catch (Exception e)
            {
              
                Console.WriteLine("[polhemus] PlStream terminated in PlStream::read_g4().");
            }
            finally
            {
                udpClient.Close();
                udpClient = null;
            }
        }

        // read thread
        private void read_liberty()
        {
            stopListening = false;

            udpClient = new UdpClient(port);
            IPEndPoint groupEP = new IPEndPoint(IPAddress.Any, port);

            try
            {
                // create temp_active to mark slots
                bool[] temp_active = new bool[max_slots];

                // using hdr + pos + qtrn frame configuration for now
                while (!stopListening)
                {
                    byte[] receiveBytes = udpClient.Receive(ref groupEP);

                    // set slots to inactive
                    for (var i = 0; i < max_slots; ++i)
                        temp_active[i] = false;

                    // offset into buffer
                    int offset = 0;
                    while (offset + 40 <= receiveBytes.Length)
                    {
                        // process header (8 bytes)
                        int nSenID = System.Convert.ToInt32(receiveBytes[offset + 2]) - 1;
                        offset += 8;

                        if (nSenID > max_slots)
                        {
                            Console.WriteLine("[polhemus] SenID is greater than" + max_sensors.ToString() + ".");
                            throw new Exception("[polhemus] SenID is greater than" + max_sensors.ToString() + ".");
                        }

                        // process stylus (4 bytes)
                        uint bfStylus = BitConverter.ToUInt32(receiveBytes, offset);
                        offset += 4;

                        // process position (12 bytes)
                        float t = BitConverter.ToSingle(receiveBytes, offset);
                        float u = BitConverter.ToSingle(receiveBytes, offset + 4);
                        float v = BitConverter.ToSingle(receiveBytes, offset + 8);
                        offset += 12;

                        // process orientation (16 bytes)
                        float w = BitConverter.ToSingle(receiveBytes, offset);
                        float x = BitConverter.ToSingle(receiveBytes, offset + 4);
                        float y = BitConverter.ToSingle(receiveBytes, offset + 8);
                        float z = BitConverter.ToSingle(receiveBytes, offset + 12);
                        offset += 16;

                        // store results
                        temp_active[nSenID] = true;
                        digio[nSenID] = bfStylus;
                        positions[nSenID] = new Vector3(t, u, v);
                        orientations[nSenID] = new Vector4(w, x, y, z);
                        
                        
                    }

                    // mark active slots
                    for (var i = 0; i < max_slots; ++i)
                        active[i] = temp_active[i];
                }
            }
            catch (Exception e)
            {
                
                Console.WriteLine("[polhemus] PlStream terminated in PlStream::read_liberty().");
            }
            finally
            {
                udpClient.Close();
                udpClient = null;
            }
        }

        // read thread
        private void read_fastrak()
        {
            stopListening = false;

            udpClient = new UdpClient(port);
            IPEndPoint groupEP = new IPEndPoint(IPAddress.Any, port);

            try
            {
                // create temp_active to mark slots
                bool[] temp_active = new bool[max_slots];

                // using hdr + pos + qtrn frame configuration for now
                var update = 0;
                while (!stopListening)
                {
                    ++update;
                    byte[] receiveBytes = udpClient.Receive(ref groupEP);

                    // set slots to inactive
                    // disabled active tracking for fastrak because of the way it sends out its data:
                    // a frame per sensor. Fastrak is a wired system, so all sensors will always be
                    // active anyways
                    /*for (var i = 0; i < max_slots; ++i)
                        temp_active[i] = false;*/

                    // offset into buffer
                    int offset = 0;
                    while (offset + 33 <= receiveBytes.Length)
                    {
                        // process header (3 bytes)
                        uint nSenID = System.Convert.ToUInt32(receiveBytes[offset + 1]) - System.Convert.ToUInt32('1');
                        offset += 3;

                        if (nSenID > max_slots)
                        {
                            Console.WriteLine("[polhemus] SenID is greater than" + max_sensors.ToString() + ".");
                            throw new Exception("[polhemus] SenID is greater than" + max_sensors.ToString() + ".");
                        }

                        // process stylus (2 bytes: space and '0' or '1')
                        uint bfStylus = System.Convert.ToUInt32(receiveBytes[offset + 1]) - System.Convert.ToUInt32('0'); ;
                        offset += 2;

                        // process position (12 bytes)
                        float t = BitConverter.ToSingle(receiveBytes, offset);
                        float u = BitConverter.ToSingle(receiveBytes, offset + 4);
                        float v = BitConverter.ToSingle(receiveBytes, offset + 8);
                        offset += 12;

                        // process orientation (16 bytes)
                        float w = BitConverter.ToSingle(receiveBytes, offset);
                        float x = BitConverter.ToSingle(receiveBytes, offset + 4);
                        float y = BitConverter.ToSingle(receiveBytes, offset + 8);
                        float z = BitConverter.ToSingle(receiveBytes, offset + 12);
                        offset += 16;

                        // store results
                        temp_active[nSenID] = true;
                        digio[nSenID] = bfStylus;
                        positions[nSenID] = new Vector3(t, u, v);
                        orientations[nSenID] = new Vector4(w, x, y, z);
                    }

                    // mark active slots (always marking as active)
                    for (var i = 0; i < max_slots; ++i)
                        active[i] = temp_active[i];
                }
            }
            catch (Exception e)
            {
               
                Console.WriteLine("[polhemus] PlStream terminated in PlStream::read_liberty().");
            }
            finally
            {
                udpClient.Close();
                udpClient = null;
            }
        }

        // cleanup
        public void StopPolhemus()
        {
            try
            {
                // signal shutdown
                stopListening = true;

                // attempt to join for 500ms
                if (!conThread.Join(500))
                {
                    // force shutdown
                    conThread.Abort();
                    if (udpClient != null)
                    {
                        udpClient.Close();
                        udpClient = null;
                    }
                }
            }
            catch (Exception e)
            {
                
            }
        }
    }
}