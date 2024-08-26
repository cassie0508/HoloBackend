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

        private IEnumerator CameraCapture()
        {
            /* Import playback file */
            try
            {
                playback = new Playback("Assets/capture.mkv");
            }
            catch (Exception ex)
            {
                Debug.LogError("Failed to open playback file: " + ex.Message);
                yield break;
            }

            // For debugging: Set up textures
            SetupTextures(ref ColorImage, ref DepthImage, ref ColorInDepthImage);

            /* Publish Camera Data */
            var calibration = playback.playback_calibration;
            byte[] xyLookupData = GenerateXYTableData(calibration);

            var extrinsics = calibration.DeviceExtrinsics[(int)CalibrationDeviceType.Depth + (int)CalibrationDeviceType.Color];
            Matrix4x4 extrinsics4x4 = new Matrix4x4();
            extrinsics4x4.SetRow(0, new Vector4(extrinsics.Rotation[0], extrinsics.Rotation[3], extrinsics.Rotation[6], extrinsics.Translation[0] / 1000.0f));
            extrinsics4x4.SetRow(1, new Vector4(extrinsics.Rotation[1], extrinsics.Rotation[4], extrinsics.Rotation[7], extrinsics.Translation[1] / 1000.0f));
            extrinsics4x4.SetRow(2, new Vector4(extrinsics.Rotation[2], extrinsics.Rotation[5], extrinsics.Rotation[8], extrinsics.Translation[2] / 1000.0f));
            extrinsics4x4.SetRow(3, new Vector4(0, 0, 0, 1));
            byte[] calibrationData = Matrix4x4ToByteArray(extrinsics4x4);

            byte[] cameraSizeData = null;
            try
            {
                using (var capture = playback.GetNextCapture())
                {
                    int[] captureArray = new int[6] {
                        1920, 1080,     // TODO: Have issue in capture.Color
                        capture.Depth.WidthPixels, capture.Depth.HeightPixels,
                        capture.IR.WidthPixels, capture.IR.HeightPixels
                    };

                    cameraSizeData = new byte[captureArray.Length * sizeof(int)];
                    Buffer.BlockCopy(captureArray, 0, cameraSizeData, 0, cameraSizeData.Length);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning("Failed to get capture: " + ex.Message);
            }

            // Data: [xyLookupData.Length][calibrationData.Length][cameraSizeData.Length]
            //       [xyLookupData][calibrationData][cameraSizeData]
            int cameraTotalSize = sizeof(int) * 3 + xyLookupData.Length + calibrationData.Length + cameraSizeData.Length;
            byte[] cameraData = new byte[cameraTotalSize];

            Buffer.BlockCopy(BitConverter.GetBytes(xyLookupData.Length), 0, cameraData, 0, sizeof(int));
            Buffer.BlockCopy(BitConverter.GetBytes(calibrationData.Length), 0, cameraData, sizeof(int) * 1, sizeof(int));
            Buffer.BlockCopy(BitConverter.GetBytes(cameraSizeData.Length), 0, cameraData, sizeof(int) * 2, sizeof(int));
             
            Buffer.BlockCopy(xyLookupData, 0, cameraData, sizeof(int) * 3, xyLookupData.Length);
            Buffer.BlockCopy(calibrationData, 0, cameraData, sizeof(int) * 3 + xyLookupData.Length, calibrationData.Length);
            Buffer.BlockCopy(cameraSizeData, 0, cameraData, sizeof(int) * 3 + xyLookupData.Length + calibrationData.Length, cameraSizeData.Length);

            PublishData("Camera", cameraData);

            /* Publish Frame Data */
            // TODO: _Device.GetCalibration(DepthMode.NFOV_2x2Binned, ColorResolution.R1080p).CreateTransformation();
            var kinectCalibration = calibration.CreateTransformation();

            while (true)
            {
                using (var capture = playback.GetNextCapture())
                {
                    if (capture == null)
                    {
                        Debug.Log("End of playback file reached.");
                        break;
                    }

                    // For debugging: Apply data to textures
                    ColorImage.LoadRawTextureData(capture.Color.Memory.ToArray());
                    ColorImage.Apply();
                    DepthImage.LoadRawTextureData(capture.Depth.Memory.ToArray());
                    DepthImage.Apply();
                    ColorInDepthImage.LoadRawTextureData(kinectCalibration.ColorImageToDepthCamera(capture).Memory.ToArray());
                    ColorInDepthImage.Apply();

                    byte[] colorData = capture.Color.Memory.ToArray();
                    byte[] depthData = capture.Depth.Memory.ToArray();
                    byte[] colorInDepthData = kinectCalibration.ColorImageToDepthCamera(capture).Memory.ToArray();

                    // Data: [colorData.Length][depthData.Length][colorInDepthData.Length]
                    //       [colorData][depthData][colorInDepthData]
                    int frameTotalSize = colorData.Length + depthData.Length + colorInDepthData.Length + sizeof(int) * 3;
                    byte[] frameData = new byte[frameTotalSize];

                    Buffer.BlockCopy(BitConverter.GetBytes(colorData.Length), 0, frameData, 0, sizeof(int));
                    Buffer.BlockCopy(BitConverter.GetBytes(depthData.Length), 0, frameData, sizeof(int) * 1, sizeof(int));
                    Buffer.BlockCopy(BitConverter.GetBytes(colorInDepthData.Length), 0, frameData, sizeof(int) * 2, sizeof(int));

                    Buffer.BlockCopy(colorData, 0, frameData, sizeof(int) * 3, colorData.Length);
                    Buffer.BlockCopy(depthData, 0, frameData, sizeof(int) * 3 + colorData.Length, depthData.Length);
                    Buffer.BlockCopy(colorInDepthData, 0, frameData, sizeof(int) * 3 + colorData.Length + depthData.Length, colorInDepthData.Length);
 
                    PublishData("Frame", frameData);
                }

                yield return null;
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
            byte[] xyTableData = xylookup.GetRawTextureData();
            return xyTableData;
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