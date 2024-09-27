using System;
using System.Collections;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;
using NetMQ;
using NetMQ.Sockets;
using System.Linq;
using UnityEngine.Playables;

namespace Kinect4Azure
{
    public class KinectPublisher : MonoBehaviour
    {
        public static KinectPublisher Instance;

        [Header("ReadOnly and exposed for Debugging")]
        [SerializeField] private Texture2D DepthImage;
        [SerializeField] private Texture2D ColorInDepthImage;
        [SerializeField] private Texture2D ResizedDepthImage;
        [SerializeField] private Texture2D ResizedColorInDepthImage;

        private Device _Device;
        private PublisherSocket dataPubSocket;
        [SerializeField] private string port = "55555";

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
                Debug.Log("Successfully bound socket port " + port);
            }
            catch (Exception ex)
            {
                Debug.LogError($"Failed to bind socket: {ex.Message}");
            }
        }

        private IEnumerator CameraCapture()
        {
            if (Device.GetInstalledCount() == 0)
            {
                Debug.LogError("No Kinect Device Found");
                yield break;
            }

            try
            {
                _Device = Device.Open();
            }
            catch (AzureKinectOpenDeviceException ex)
            {
                Debug.LogError($"Failed to open Azure Kinect device: {ex.Message}");
                yield break;
            }

            var configuration = new DeviceConfiguration
            {
                ColorFormat = ImageFormat.ColorBGRA32,
                ColorResolution = ColorResolution.R1080p,
                DepthMode = DepthMode.NFOV_2x2Binned,
                SynchronizedImagesOnly = true,
                CameraFPS = FPS.FPS30
            };

            _Device.StartCameras(configuration);

            // For debugging: Set up textures
            SetupTextures(ref DepthImage, ref ColorInDepthImage, ref ResizedDepthImage, ref ResizedColorInDepthImage);


            /* Publish Camera Data */
            var extrinsics = _Device.GetCalibration().DeviceExtrinsics[(int)CalibrationDeviceType.Depth + (int)CalibrationDeviceType.Color];
            Matrix4x4 extrinsics4x4 = new Matrix4x4();
            extrinsics4x4.SetRow(0, new Vector4(extrinsics.Rotation[0], extrinsics.Rotation[3], extrinsics.Rotation[6], extrinsics.Translation[0] / 1000.0f));
            extrinsics4x4.SetRow(1, new Vector4(extrinsics.Rotation[1], extrinsics.Rotation[4], extrinsics.Rotation[7], extrinsics.Translation[1] / 1000.0f));
            extrinsics4x4.SetRow(2, new Vector4(extrinsics.Rotation[2], extrinsics.Rotation[5], extrinsics.Rotation[8], extrinsics.Translation[2] / 1000.0f));
            extrinsics4x4.SetRow(3, new Vector4(0, 0, 0, 1));
            byte[] calibrationData = Matrix4x4ToByteArray(extrinsics4x4);

            byte[] cameraSizeData = null;
            try
            {
                using (var capture = _Device.GetCapture())
                {
                    int[] captureArray = new int[6] {
                        capture.Color.WidthPixels / 2, capture.Color.HeightPixels / 2,
                        capture.Depth.WidthPixels / 2, capture.Depth.HeightPixels / 2,
                        capture.IR.WidthPixels / 2, capture.IR.HeightPixels / 2
                    };

                    cameraSizeData = new byte[captureArray.Length * sizeof(int)];
                    Buffer.BlockCopy(captureArray, 0, cameraSizeData, 0, cameraSizeData.Length);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning("Failed to get capture: " + ex.Message);
            }

            // Data: [calibrationData.Length][cameraSizeData.Length]
            //       [calibrationData][cameraSizeData]
            int cameraTotalSize = sizeof(int) * 2 + calibrationData.Length + cameraSizeData.Length;
            byte[] cameraData = new byte[cameraTotalSize];

            Buffer.BlockCopy(BitConverter.GetBytes(calibrationData.Length), 0, cameraData, 0, sizeof(int));
            Buffer.BlockCopy(BitConverter.GetBytes(cameraSizeData.Length), 0, cameraData, sizeof(int) * 1, sizeof(int));

            Buffer.BlockCopy(calibrationData, 0, cameraData, sizeof(int) * 2, calibrationData.Length);
            Buffer.BlockCopy(cameraSizeData, 0, cameraData, sizeof(int) * 2 + calibrationData.Length, cameraSizeData.Length);

            PublishData("Camera", cameraData);

            /* Publish xyLookupData */
            byte[] xyLookupData = GenerateXYTableData();
            PublishData($"Lookup", xyLookupData);

            /* Publish Frame Data */
            var kinectCalibration = _Device.GetCalibration(DepthMode.NFOV_2x2Binned, ColorResolution.R1080p).CreateTransformation();

            while (true)
            {
                using (var capture = _Device.GetCapture())
                {
                    // For debugging: Apply data to textures
                    byte[] depthData = capture.Depth.Memory.ToArray();
                    byte[] colorInDepthData = kinectCalibration.ColorImageToDepthCamera(capture).Memory.ToArray();
                    DepthImage.LoadRawTextureData(depthData);
                    DepthImage.Apply();
                    ColorInDepthImage.LoadRawTextureData(colorInDepthData);
                    ColorInDepthImage.Apply();

                    // Resize both depth and color-in-depth images
                    ResizeTexture(DepthImage, ResizedDepthImage);
                    ResizeTexture(ColorInDepthImage, ResizedColorInDepthImage);

                    // Get the resized data
                    byte[] resizedDepthData = ResizedDepthImage.GetRawTextureData();
                    byte[] resizedColorInDepthData = ResizedColorInDepthImage.GetRawTextureData();

                    // Create a buffer for the frame data
                    int frameTotalSize = resizedDepthData.Length + resizedColorInDepthData.Length + sizeof(int) * 2;
                    byte[] frameData = new byte[frameTotalSize];

                    Buffer.BlockCopy(BitConverter.GetBytes(resizedDepthData.Length), 0, frameData, 0, sizeof(int));
                    Buffer.BlockCopy(BitConverter.GetBytes(resizedColorInDepthData.Length), 0, frameData, sizeof(int), sizeof(int));
                    Buffer.BlockCopy(resizedDepthData, 0, frameData, sizeof(int) * 2, resizedDepthData.Length);
                    Buffer.BlockCopy(resizedColorInDepthData, 0, frameData, sizeof(int) * 2 + resizedDepthData.Length, resizedColorInDepthData.Length);

                    PublishData("Frame", frameData);
                }

                yield return null;
            }
        }

        private void ResizeTexture(Texture2D source, Texture2D target)
        {
            RenderTexture rt = RenderTexture.GetTemporary(target.width, target.height);
            RenderTexture previous = RenderTexture.active; // Store the current active RenderTexture

            Graphics.Blit(source, rt);
            RenderTexture.active = rt;
            target.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
            target.Apply();

            RenderTexture.active = previous; // Restore the previous active RenderTexture
            RenderTexture.ReleaseTemporary(rt);
        }


        private void SetupTextures(ref Texture2D Depth, ref Texture2D ColorInDepth, ref Texture2D ResizedDepth, ref Texture2D ResizedColorInDepth)
        {
            try
            {
                using (var capture = _Device.GetCapture())
                {
                    if (Depth == null)
                        Depth = new Texture2D(capture.Depth.WidthPixels, capture.Depth.HeightPixels, TextureFormat.R16, false);
                    if (ColorInDepth == null)
                        ColorInDepth = new Texture2D(capture.IR.WidthPixels, capture.IR.HeightPixels, TextureFormat.BGRA32, false);
                    if (ResizedDepth == null)
                        ResizedDepth = new Texture2D(capture.Depth.WidthPixels / 2, capture.Depth.HeightPixels / 2, TextureFormat.R16, false);
                    if (ResizedColorInDepth == null)
                        ResizedColorInDepth = new Texture2D(capture.IR.WidthPixels / 2, capture.IR.HeightPixels / 2, TextureFormat.BGRA32, false);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"An error occurred " + ex.Message);
            }
        }

        private byte[] GenerateXYTableData()
        {
            var cal = _Device.GetCalibration();
            Texture2D xylookup = new Texture2D(ResizedDepthImage.width, ResizedDepthImage.height, TextureFormat.RGBAFloat, false);
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

            if (_Device != null)
            {
                _Device.StopCameras();
                _Device.Dispose();
            }
        }
    }
}