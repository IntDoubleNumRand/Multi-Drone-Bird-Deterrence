// Assets/Scripts/DroneVisual.cs
using UnityEngine;
using ROS2;
using geometry_msgs.msg;
using Vector3 = UnityEngine.Vector3;

public class DroneVisual : ROS2UnityComponent
{
    private ROS2Node ros2Node;
    private GameObject drone;

    private readonly object poseLock = new object();
    private bool hasPose;
    private Vector3 latestPos;

    void Start()
    {
        drone = transform.GetChild(0).gameObject;
        ros2Node = CreateNode("unity_drone_viz");

        var sensorQos = new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);
        ros2Node.CreateSubscription<PoseStamped>(
            "/mavros/local_position/pose",
            OnPose,
            sensorQos);
    }

    void OnPose(PoseStamped msg)
    {
        var p = msg.Pose.Position;
        var mapped = FieldLayout.RosToUnity((float)p.X, (float)p.Y, (float)p.Z);

        lock (poseLock)
        {
            latestPos = mapped;
            hasPose = true;
        }
    }

    void Update()
    {
        if (!hasPose) return;

        Vector3 p;
        lock (poseLock)
            p = latestPos;

        drone.transform.position = p;
    }
}
