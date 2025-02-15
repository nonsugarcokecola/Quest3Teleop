using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;  

public class ROSRawImageSubscriber : MonoBehaviour
{
    public RawImage MidCamera; 
    public RawImage LeftCamera; 
    public RawImage RightCamera; 
    private ROSConnection ros;
    private Texture2D MidTexturePool;
    private Texture2D LeftTexturePool;
    private Texture2D RightTexturePool;

    private string MidtopicName = "/mid_cam/compressed";
    private string LefttopicName = "/left_cam/compressed";
    private string RighttopicName = "/right_cam/compressed";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImageMsg>(MidtopicName, MidOnImageReceived);
        ros.Subscribe<CompressedImageMsg>(LefttopicName, LeftOnImageReceived);
        ros.Subscribe<CompressedImageMsg>(RighttopicName, RightOnImageReceived);
        MidTexturePool = new Texture2D(640, 480, TextureFormat.RGB24, false);
        LeftTexturePool = new Texture2D(640, 480, TextureFormat.RGB24, false);
        RightTexturePool = new Texture2D(640, 480, TextureFormat.RGB24, false);
    }

    private void MidOnImageReceived(CompressedImageMsg msg)
    {
        MidTexturePool.LoadImage(msg.data);
        MidTexturePool.Apply();
        if (MidTexturePool != null) {
            MidCamera.texture = MidTexturePool;
        }
    }

    private void LeftOnImageReceived(CompressedImageMsg msg)
    {
        LeftTexturePool.LoadImage(msg.data);
        LeftTexturePool.Apply();
        if (LeftTexturePool != null) {
            LeftCamera.texture = LeftTexturePool;
        }
    }

    private void RightOnImageReceived(CompressedImageMsg msg)
    {
        RightTexturePool.LoadImage(msg.data);
        RightTexturePool.Apply();
        if (RightTexturePool != null) {
            RightCamera.texture = RightTexturePool;
        }
    }
}