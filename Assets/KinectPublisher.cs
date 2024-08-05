using System;
using System.Collections;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;
using NetMQ;
using NetMQ.Sockets;

namespace Kinect4Azure
{
    public class KinectPublisher : MonoBehaviour
    {
        public static KinectPublisher Instance;

        [Header("Pointcloud Configs")]
        public bool UseOcclusionShader = true;
        public Shader PointCloudShader;
        public Shader OcclusionShader;
        [Range(0.01f, 0.1f)]
        public float MaxPointDistance = 0.02f;

        [Header("Background Configs\n(Only works if this script is attached onto the camera)")]
        public bool EnableARBackground = true;
        [Tooltip("Only needs to be set when BlitToCamera is checked")]
        public Material ARBackgroundMaterial;

        [Header("ReadOnly and exposed for Debugging")]
        [SerializeField] private Texture2D DepthImage;
        [SerializeField] private Texture2D ColorImage;
        [SerializeField] private Texture2D ColorInDepthImage;

        private Device _Device;
        private PublisherSocket dataPubSocket;
        [SerializeField] private string port = "12345";
        //private bool socketInitialized = false;

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
                //socketInitialized = true;
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
                using (var capture = _Device.GetCapture())
                {
                    if (Color == null)
                        Color = new Texture2D(capture.Color.WidthPixels, capture.Color.HeightPixels, TextureFormat.BGRA32, false);
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
            if (Device.GetInstalledCount() == 0)
            {
                Debug.LogError("No Kinect Device Found");
                yield break;
            }

            try
            {
                _Device = Device.Open(0); // Open the first connected device (index 0)
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

            var kinectCalibration = _Device.GetCalibration(DepthMode.NFOV_2x2Binned, ColorResolution.R1080p).CreateTransformation();

            SetupTextures(ref ColorImage, ref DepthImage, ref ColorInDepthImage);

            while (true)
            {
                using (var capture = _Device.GetCapture())
                {
                    ColorImage.LoadRawTextureData(capture.Color.Memory.ToArray());
                    ColorImage.Apply();

                    DepthImage.LoadRawTextureData(capture.Depth.Memory.ToArray());
                    DepthImage.Apply();

                    ColorInDepthImage.LoadRawTextureData(kinectCalibration.ColorImageToDepthCamera(capture).Memory.ToArray());
                    ColorInDepthImage.Apply();

                    // Publish color data
                    byte[] colorData = ColorImage.GetRawTextureData();
                    PublishData("ColorFrame", colorData);

                    // Publish depth data
                    byte[] depthData = DepthImage.GetRawTextureData();
                    PublishData("DepthFrame", depthData);

                    // Publish color in depth data
                    byte[] colorInDepthData = ColorInDepthImage.GetRawTextureData();
                    PublishData("ColorInDepthFrame", colorInDepthData);
                }
                yield return null;
            }
        }

        private void OnDestroy()
        {
            Debug.Log("Closing socket on port " + port);
            dataPubSocket.Dispose();
            NetMQConfig.Cleanup(false);
            dataPubSocket = null;

            if (_Device != null)
            {
                _Device.StopCameras();
                _Device.Dispose();
            }

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
    }
}
