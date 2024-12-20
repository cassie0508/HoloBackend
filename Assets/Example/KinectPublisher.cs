﻿using System;
using System.Collections;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;
using NetMQ;
using NetMQ.Sockets;
using System.Linq;
using UnityEngine.Playables;
using PubSub;

public class KinectPublisher : MonoBehaviour
{
    public enum Mode
    {
        Replica,
        Mirror
    }

    [Header("Visualization Mode")]
    public Mode CurrentMode;

    [Header("Network Settings")]
    [SerializeField] private string port = "12345";
    private PublisherSocket dataPubSocket;

    [Header("Replica Mode Settings")]
    [SerializeField] private Texture2D DepthImage;
    [SerializeField] private Texture2D ColorInDepthImage;
    
    [Header("Mirror Mode Settings")]
    [SerializeField] private Texture2D ColorImage;
    private Texture2D resizedColorImage;

    private Device _Device;

    private void Start()
    {
        InitializeSocket();

        if(CurrentMode == Mode.Replica) StartCoroutine(CameraCaptureReplica());
        else if (CurrentMode == Mode.Mirror) StartCoroutine(CameraCaptureMirror());
    }

    private void InitializeSocket()
    {
        try
        {
            AsyncIO.ForceDotNet.Force();
            dataPubSocket = new PublisherSocket();
            dataPubSocket.Options.SendHighWatermark = 10;

            dataPubSocket.Bind($"tcp://*:{port}");
            Debug.Log("Successfully bound socket port " + port);
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to bind socket: {ex.Message}");
        }
    }

    private IEnumerator CameraCaptureMirror()
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
        if (!SetupTextures(ref ColorImage, ref resizedColorImage, ref DepthImage, ref ColorInDepthImage))
        {
            Debug.LogError("KinectPublisher::CameraCapture(): Something went wrong while setting up camera textures");
            yield break;
        }

        int[] sizeArray = new int[2] { resizedColorImage.width, resizedColorImage.height };
        byte[] sizeData = new byte[sizeArray.Length * sizeof(int)];
        Buffer.BlockCopy(sizeArray, 0, sizeData, 0, sizeData.Length);
        PublishData("Size", sizeData);

        /* Publish Frame Data */
        while (true)
        {
            using (var capture = _Device.GetCapture())
            {
                byte[] colorData = capture.Color.Memory.ToArray();

                ColorImage.LoadRawTextureData(colorData);
                ColorImage.Apply();

                DownsampleTexture(ref ColorImage, ref resizedColorImage, resizedColorImage.width, resizedColorImage.height);
                byte[] resizedColorData = resizedColorImage.GetRawTextureData();

                PublishData("Frame", resizedColorData);
            }

            //yield return new WaitForSeconds(0.2f); //5 frames per second
            yield return null;
        }
    }

    private IEnumerator CameraCaptureReplica()
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
        if (!SetupTextures(ref ColorImage, ref resizedColorImage, ref DepthImage, ref ColorInDepthImage))
        {
            Debug.LogError("KinectPublisher::CameraCapture(): Something went wrong while setting up camera textures");
            yield break;
        }

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
                    capture.Color.WidthPixels, capture.Color.HeightPixels,
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
        Debug.Log($"Lookup data length is {xyLookupData.Length}");
        int splitSize = 500000;  // Split xyLookupData into 3 parts, each part is 50000 in length
        byte[][] xyLookupParts = new byte[3][];

        for (int i = 0; i < 3; i++)
        {
            int startIdx = i * splitSize;
            int length = Mathf.Min(splitSize, xyLookupData.Length - startIdx);
            xyLookupParts[i] = new byte[length];
            Array.Copy(xyLookupData, startIdx, xyLookupParts[i], 0, length);
            PublishData($"Lookup{i + 1}", xyLookupParts[i]);
        }

        /* Publish Frame Data */
        var kinectCalibration = _Device.GetCalibration(DepthMode.NFOV_2x2Binned, ColorResolution.R1080p).CreateTransformation();

        while (true)
        {
            using (var capture = _Device.GetCapture())
            {
                byte[] colorData = capture.Color.Memory.ToArray();
                byte[] depthData = capture.Depth.Memory.ToArray();
                byte[] colorInDepthData = kinectCalibration.ColorImageToDepthCamera(capture).Memory.ToArray();

                Debug.Log($"colorData is {colorData.Length}, depthData is {depthData.Length}, colorInDepthData is {colorInDepthData.Length}");

                DepthImage.LoadRawTextureData(depthData);
                DepthImage.Apply();
                ColorInDepthImage.LoadRawTextureData(colorInDepthData);
                ColorInDepthImage.Apply();

                //byte[] compressedColorInDepthData = ColorInDepthImage.EncodeToJPG(50);

                int frameTotalSize = depthData.Length + colorInDepthData.Length + sizeof(int) * 2;
                byte[] frameData = new byte[frameTotalSize];

                Buffer.BlockCopy(BitConverter.GetBytes(depthData.Length), 0, frameData, 0, sizeof(int));
                Buffer.BlockCopy(BitConverter.GetBytes(colorInDepthData.Length), 0, frameData, sizeof(int), sizeof(int));

                Buffer.BlockCopy(depthData, 0, frameData, sizeof(int) * 2, depthData.Length);
                Buffer.BlockCopy(colorInDepthData, 0, frameData, sizeof(int) * 2 + depthData.Length, colorInDepthData.Length);

                PublishData("Frame", frameData);
            }

            //yield return new WaitForSeconds(0.2f); //5 frames per second
            yield return null;
        }
    }

    private bool SetupTextures(ref Texture2D Color, ref Texture2D resizedColor, ref Texture2D Depth, ref Texture2D ColorInDepth)
    {
        try
        {
            using (var capture = _Device.GetCapture())
            {
                if (Color == null)
                    Color = new Texture2D(capture.Color.WidthPixels, capture.Color.HeightPixels, TextureFormat.BGRA32, false);
                if (resizedColor == null)
                    resizedColor = new Texture2D(capture.Color.WidthPixels/4, capture.Color.HeightPixels/4, TextureFormat.BGRA32, false);
                if (Depth == null)
                    Depth = new Texture2D(capture.Depth.WidthPixels, capture.Depth.HeightPixels, TextureFormat.R16, false);
                if (ColorInDepth == null)
                    ColorInDepth = new Texture2D(capture.IR.WidthPixels, capture.IR.HeightPixels, TextureFormat.BGRA32, false);
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"An error occurred " + ex.Message);
            return false;
        }
        return true;
    }

    public void DownsampleTexture(ref Texture2D originalTexture, ref Texture2D resizedTexture, int targetWidth, int targetHeight)
    {
        RenderTexture rt = new RenderTexture(targetWidth, targetHeight, 24);
        RenderTexture.active = rt;

        // Copy original texture to the render texture
        Graphics.Blit(originalTexture, rt);

        // Read pixels from the render texture into the new Texture2D
        resizedTexture.ReadPixels(new Rect(0, 0, targetWidth, targetHeight), 0, 0);
        resizedTexture.Apply();

        // Clean up
        RenderTexture.active = null;
        rt.Release();
    }


    private byte[] GenerateXYTableData()
    {
        var cal = _Device.GetCalibration();
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
                Debug.Log($"Sending topic and data length: {topic}, {data.Length}");
                if(topic == "Frame")
                {
                    long timestamp = DateTime.UtcNow.Ticks;
                    byte[] timestampBytes = BitConverter.GetBytes(timestamp);

                    byte[] message = new byte[timestampBytes.Length + data.Length];
                    Buffer.BlockCopy(timestampBytes, 0, message, 0, timestampBytes.Length);
                    Buffer.BlockCopy(data, 0, message, timestampBytes.Length, data.Length);
                    dataPubSocket.SendMoreFrame(topic).SendFrame(message);
                }
                else
                {
                    dataPubSocket.SendMoreFrame(topic).SendFrame(data);
                }
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