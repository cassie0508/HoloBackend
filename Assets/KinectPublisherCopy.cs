using System;
using System.Collections;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;
using NetMQ;
using NetMQ.Sockets;
using System.Linq;

namespace Kinect4Azure
{
    public class KinectPublisherCopy : MonoBehaviour
    {
        public static KinectPublisherCopy Instance;

        [Header("ReadOnly and exposed for Debugging")]
        [SerializeField] private Texture2D DepthImage;
        [SerializeField] private Texture2D ColorImage;
        [SerializeField] private Texture2D ColorInDepthImage;
        [SerializeField] private int[] Capture;
        [SerializeField] private Texture2D LookUp;

        private Playback playback;
        private PublisherSocket dataPubSocket;
        [SerializeField] private string port = "12345";

        private void Awake()
        {
            Instance = this;
            InitializeSocket();
            StartCoroutine(CameraCapture());
        }

        private void InitializeSocket()
        {
            try
            {
                AsyncIO.ForceDotNet.Force();
                dataPubSocket = new PublisherSocket();
                dataPubSocket.Options.TcpKeepalive = false;
                dataPubSocket.Options.SendHighWatermark = 10;

                dataPubSocket.Bind($"tcp://*:{port}");
                Debug.Log("Successfully bound socket to port " + port);
            }
            catch (Exception ex)
            {
                Debug.LogError($"Failed to bind socket: {ex.Message}");
            }
        }

        private void SetupTextures(ref Texture2D Color, ref Texture2D Depth, ref Texture2D ColorInDepth)
        {
            try
            {
                using (var capture = playback.GetNextCapture())
                {
                    if (Color == null)
                        Color = new Texture2D(1920, 1080, TextureFormat.BGRA32, false);
                    if (Depth == null)
                        Depth = new Texture2D(capture.Depth.WidthPixels, capture.Depth.HeightPixels, TextureFormat.R16, false);
                    if (ColorInDepth == null)
                        ColorInDepth = new Texture2D(capture.IR.WidthPixels, capture.IR.HeightPixels, TextureFormat.BGRA32, false);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"An error occurred " + ex.Message);
            }
        }

        private IEnumerator CameraCapture()
        {
            try
            {
                playback = new Playback("Assets/capture.mkv");
            }
            catch (Exception ex)
            {
                Debug.LogError($"Failed to open playback file: {ex.Message}");
                yield break;
            }

            var calibration = playback.playback_calibration;
            var kinectCalibration = calibration.CreateTransformation();

            SetupTextures(ref ColorImage, ref DepthImage, ref ColorInDepthImage);


            // Publish camera capture information
            try
            {
                using (var capture = playback.GetNextCapture())
                {
                    Capture = new int[6] {
                        1920, 1080,
                        capture.Depth.WidthPixels, capture.Depth.HeightPixels,
                        capture.IR.WidthPixels, capture.IR.HeightPixels
                    };

                    byte[] captureData = new byte[Capture.Length * sizeof(int)];
                    Buffer.BlockCopy(Capture, 0, captureData, 0, captureData.Length);
                    PublishData("Capture", captureData);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"An error occurred " + ex.Message);
            }

            byte[] xyTableData = GenerateXYTableData(calibration);
            PublishData("XYTable", xyTableData);


            var extrinsics = calibration.DeviceExtrinsics[(int)CalibrationDeviceType.Depth + (int)CalibrationDeviceType.Color];
            Matrix4x4 extrinsics4x4 = new Matrix4x4();
            extrinsics4x4.SetRow(0, new Vector4(extrinsics.Rotation[0], extrinsics.Rotation[3], extrinsics.Rotation[6], extrinsics.Translation[0] / 1000.0f));
            extrinsics4x4.SetRow(1, new Vector4(extrinsics.Rotation[1], extrinsics.Rotation[4], extrinsics.Rotation[7], extrinsics.Translation[1] / 1000.0f));
            extrinsics4x4.SetRow(2, new Vector4(extrinsics.Rotation[2], extrinsics.Rotation[5], extrinsics.Rotation[8], extrinsics.Translation[2] / 1000.0f));
            extrinsics4x4.SetRow(3, new Vector4(0, 0, 0, 1));

            Matrix4x4 color2DepthCalibration = extrinsics4x4;

            byte[] calibrationData = Matrix4x4ToByteArray(color2DepthCalibration);
            PublishData("Color2DepthCalibration", calibrationData);

            while (true)
            {
                using (var capture = playback.GetNextCapture())
                {
                    if (capture == null)
                    {
                        Debug.Log("End of playback file reached.");
                        break;
                    }

                    ColorImage.LoadRawTextureData(capture.Color.Memory.ToArray());
                    ColorImage.Apply();

                    DepthImage.LoadRawTextureData(capture.Depth.Memory.ToArray());
                    DepthImage.Apply();

                    ColorInDepthImage.LoadRawTextureData(kinectCalibration.ColorImageToDepthCamera(capture).Memory.ToArray());
                    ColorInDepthImage.Apply();

                    byte[] colorData = ColorImage.GetRawTextureData();
                    byte[] depthData = DepthImage.GetRawTextureData();
                    byte[] colorInDepthData = ColorInDepthImage.GetRawTextureData();

                    int totalSize = colorData.Length + depthData.Length + colorInDepthData.Length + sizeof(int) * 3;
                    byte[] allData = new byte[totalSize];

                    Buffer.BlockCopy(BitConverter.GetBytes(colorData.Length), 0, allData, 0, sizeof(int));
                    Buffer.BlockCopy(BitConverter.GetBytes(depthData.Length), 0, allData, sizeof(int), sizeof(int));
                    Buffer.BlockCopy(BitConverter.GetBytes(colorInDepthData.Length), 0, allData, sizeof(int) * 2, sizeof(int));

                    Buffer.BlockCopy(colorData, 0, allData, sizeof(int) * 3, colorData.Length);
                    Buffer.BlockCopy(depthData, 0, allData, sizeof(int) * 3 + colorData.Length, depthData.Length);
                    Buffer.BlockCopy(colorInDepthData, 0, allData, sizeof(int) * 3 + colorData.Length + depthData.Length, colorInDepthData.Length);

                    PublishData("AllFrames", allData);
                }
                yield return null;
            }
        }

        private byte[] Matrix4x4ToByteArray(Matrix4x4 matrix)
        {
            float[] matrixFloats = new float[16];
            for (int i = 0; i < 16; i++)
            {
                matrixFloats[i] = matrix[i];
            }

            byte[] byteArray = new byte[matrixFloats.Length * sizeof(float)];
            Buffer.BlockCopy(matrixFloats, 0, byteArray, 0, byteArray.Length);
            return byteArray;
        }

        private void PublishData(string topic, byte[] data)
        {
            if (dataPubSocket != null)
            {
                try
                {
                    dataPubSocket.SendMoreFrame(topic).SendFrame(data);
                }
                catch (NetMQ.TerminatingException)
                {
                    Debug.LogWarning("Context was terminated. Reinitializing socket.");
                    InitializeSocket();
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"Failed to publish data: {ex.Message}");
                }
            }
        }

        private byte[] GenerateXYTableData(Calibration cal)
        {
            Texture2D xylookup = new Texture2D(DepthImage.width, DepthImage.height, TextureFormat.RGBAFloat, false);
            Vector2[] data = new Vector2[xylookup.width * xylookup.height];
            int idx = 0;

            System.Numerics.Vector2 p = new System.Numerics.Vector2();
            System.Numerics.Vector3? ray;

            for (int y = 0; y < xylookup.height; y++)
            {
                p.Y = y;
                for (int x = 0; x < xylookup.width; x++)
                {
                    p.X = x;
                    ray = cal.TransformTo3D(p, 1f, CalibrationDeviceType.Depth, CalibrationDeviceType.Depth);
                    if (ray.HasValue)
                    {
                        float xf = ray.Value.X;
                        float yf = ray.Value.Y;
                        data[idx].x = xf;
                        data[idx].y = yf;
                        xylookup.SetPixel(x, y, new Color((xf + 1) / 2, (yf + 1) / 2, 0, 0));
                    }
                    else
                    {
                        xylookup.SetPixel(x, y, new Color(0, 0, 0, 0));
                        data[idx].x = 0;
                        data[idx].y = 0;
                    }
                }
            }

            xylookup.Apply();
            LookUp = xylookup;

            byte[] xyTableData = xylookup.GetRawTextureData();
            return xyTableData;
        }

        private void OnDestroy()
        {
            Debug.Log("Closing socket on port " + port);
            dataPubSocket.Dispose();
            NetMQConfig.Cleanup(false);
            dataPubSocket = null;

            StopAllCoroutines();
            Task.WaitAny(Task.Delay(1000));

            if (playback != null)
            {
                playback.ClosePlaybackFile();
                playback.Dispose();
            }
        }
    }
}