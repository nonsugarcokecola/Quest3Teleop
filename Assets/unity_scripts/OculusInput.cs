using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class OculusInput : MonoBehaviour
{
    ROSConnection ros;
    private string topicName_1 = "mmk2_left_end";
    private string topicName_2 = "mmk2_right_end";
    private string topicName_3 = "mmk2_left_gripper_release";
    private string topicName_4 = "mmk2_left_gripper_tighten";
    private string topicName_5 = "mmk2_right_gripper_release";
    private string topicName_6 = "mmk2_right_gripper_tighten";
    // public string topicName_7 = "mmk2_head";
    private string topicName_8 = "mmk2_platform";
    private string topicName_9 = "mmk2_linear";
    private string topicName_10 = "mmk2_angular";

    public float publishMessageFrequency = 0.1f;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName_1);
        ros.RegisterPublisher<PoseMsg>(topicName_2);
        ros.RegisterPublisher<Float32Msg>(topicName_3);
        ros.RegisterPublisher<Float32Msg>(topicName_4);
        ros.RegisterPublisher<Float32Msg>(topicName_5);
        ros.RegisterPublisher<Float32Msg>(topicName_6);
        // ros.RegisterPublisher<PosRotMsg>(topicName_7);
        ros.RegisterPublisher<Float32Msg>(topicName_8);
        ros.RegisterPublisher<Float32Msg>(topicName_9);
        ros.RegisterPublisher<Float32Msg>(topicName_10);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // 获取左右手柄的位姿
            Vector3 leftHandPos = OVRInput.GetLocalControllerPosition(OVRInput.Controller.LTouch);
            Quaternion leftHandRot = OVRInput.GetLocalControllerRotation(OVRInput.Controller.LTouch);
            Vector3 rightHandPos = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);
            Quaternion rightHandRot = OVRInput.GetLocalControllerRotation(OVRInput.Controller.RTouch);
            // Debug.Log("Left Hand Position: " + leftHandPos);
            // Debug.Log("Left Hand Rotation: " + leftHandRot);
            // Debug.Log("Right Hand Position: " + rightHandPos);
            // Debug.Log("Right Hand Rotation: " + rightHandRot);

            // 获取摇杆输入
            Vector2 leftThumbstick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
            Vector2 rightThumbstick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch);
            // Debug.Log("Left Thumbstick: " + leftThumbstick);
            // Debug.Log("Right Thumbstick: " + rightThumbstick);

            // 获取触发按钮
            float leftTrigger = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
            float rightTrigger = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);

            // 获得抓握按钮
            float leftGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch);
            float rightGrip = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.RTouch);

            // 获取右手手柄的 A 按键
            bool rightAButton = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.RTouch);
            Debug.Log("Right A Button: " + rightAButton);

            // 获取左手手柄的 X 按键
            bool leftXButton = OVRInput.Get(OVRInput.Button.One, OVRInput.Controller.LTouch);
            Debug.Log("Left X Button: " + leftXButton);

            PoseMsg leftHandPose = new PoseMsg(
                new PointMsg(leftHandPos.x, leftHandPos.y, leftHandPos.z),
                new QuaternionMsg(leftHandRot.x, leftHandRot.y, leftHandRot.z, leftHandRot.w)
            );
            PoseMsg rightHandPose = new PoseMsg(
                new PointMsg(rightHandPos.x, rightHandPos.y, rightHandPos.z),
                new QuaternionMsg(rightHandRot.x, rightHandRot.y, rightHandRot.z, rightHandRot.w)
            );
            Float32Msg leftGripperRelease = new Float32Msg(leftTrigger);
            Float32Msg leftGripperTighten = new Float32Msg(leftGrip);
            Float32Msg rightGripperRelease = new Float32Msg(rightTrigger);
            Float32Msg rightGripperTighten = new Float32Msg(rightGrip);
            Float32Msg platform = new Float32Msg(rightThumbstick.y);
            Float32Msg linear = new Float32Msg(leftThumbstick.y);
            Float32Msg angular = new Float32Msg(leftThumbstick.x);

            if (leftXButton) {
                ros.Publish(topicName_1, leftHandPose);
                Debug.Log("Left Hand Pose Published");
            }
            if (rightAButton) {
                ros.Publish(topicName_2, rightHandPose);
                Debug.Log("Right Hand Pose Published");
            }
            ros.Publish(topicName_3, leftGripperRelease);
            ros.Publish(topicName_4, leftGripperTighten);
            ros.Publish(topicName_5, rightGripperRelease);
            ros.Publish(topicName_6, rightGripperTighten);
            ros.Publish(topicName_8, platform);
            ros.Publish(topicName_9, linear);
            ros.Publish(topicName_10, angular);

            timeElapsed = 0;
        }
    }
}