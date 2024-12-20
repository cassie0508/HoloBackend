using NetMQ;
using NetMQ.Sockets;
using PubSub;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UI;

public class WebcamPublisher : MonoBehaviour
{
    public int WebcamIndex = 0;

    [Header("UI Display")]
    public RawImage ConnectionIndicator;

    [SerializeField] private Texture2D ColorImage;
    private WebCamTexture tex;

    [SerializeField] private string port = "55555";
    private PublisherSocket dataPubSocket;

    void Start()
    {
        InitializeSocket();

        WebCamDevice[] devices = WebCamTexture.devices;
        for (int i = 0; i < devices.Length; i++)
        {
            print(i + ": Webcam available: " + devices[i].name);
        }

        tex = new WebCamTexture(devices[WebcamIndex].name);
        tex.Play();

        ColorImage = new Texture2D(tex.width, tex.height, TextureFormat.RGB24, false);
        int[] sizeArray = new int[2] { ColorImage.width, ColorImage.height };
        byte[] sizeData = new byte[sizeArray.Length * sizeof(int)];
        Buffer.BlockCopy(sizeArray, 0, sizeData, 0, sizeData.Length);
        PublishData("Size", sizeData);
    }

    private void InitializeSocket()
    {
        try
        {
            AsyncIO.ForceDotNet.Force();
            dataPubSocket = new PublisherSocket();

            dataPubSocket.Bind($"tcp://*:{port}");
            Debug.Log("Successfully bound socket port " + port);

            if (ConnectionIndicator)
                ConnectionIndicator.color = Color.green;
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to bind socket: {ex.Message}");
        }
    }

    void Update()
    {
        if (tex != null && tex.isPlaying && ColorImage != null)
        {
            // Transfer WebCamTexture to Texture2D
            ColorImage.SetPixels(tex.GetPixels());
            ColorImage.Apply();

            byte[] colorData = ColorImage.GetRawTextureData();

            PublishData("Color", colorData);
        }
    }

    private void PublishData(string topic, byte[] data)
    {
        if (dataPubSocket != null)
        {
            try
            {
                long timestamp = DateTime.UtcNow.Ticks;
                byte[] timestampBytes = BitConverter.GetBytes(timestamp);

                byte[] message = new byte[timestampBytes.Length + data.Length];
                Buffer.BlockCopy(timestampBytes, 0, message, 0, timestampBytes.Length);
                Buffer.BlockCopy(data, 0, message, timestampBytes.Length, data.Length);

                dataPubSocket.SendMoreFrame(topic).SendFrame(message);
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

        if (dataPubSocket != null)
        {
            dataPubSocket.Dispose();
            NetMQConfig.Cleanup(false);
            dataPubSocket = null;
        }

        if (tex != null)
        {
            tex.Stop();
            tex = null;
        }
    }

}
