// Field layout. Scene markers override defaults; must match config/field_layout.yaml.
using System.Collections.Generic;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

[System.Serializable]
public struct BirdSpawn
{
    public float rosX;
    public float rosY;
    public float rosZ;
    public float dx;
    public float dy;
}

[System.Serializable]
public struct ObstacleSpawn
{
    public string name;
    public float rosX;
    public float rosY;
    public float radius;
}

public class FieldLayout : MonoBehaviour
{
    public float limitXY = 15f;
    [Tooltip("Bird play area (ROS field_xy); smaller than mock world limitXY")]
    public float fieldXY = 10f;
    public float birdAltitude = 4f;
    [Tooltip("Must match config/field_layout.yaml bird count")]
    public int maxBirds = 3;
    public int maxObstacles = 8;

    [Tooltip("Static obstacles (matches config/field_layout.yaml obstacles)")]
    public ObstacleSpawn[] obstacles = new ObstacleSpawn[]
    {
        new ObstacleSpawn { name = "house", rosX = -7f, rosY = -0.39f, radius = 3f },
        new ObstacleSpawn { name = "tree_nw", rosX = -5f, rosY = 12f, radius = 1.5f },
        new ObstacleSpawn { name = "rock_se", rosX = 8f, rosY = -8f, radius = 1.2f },
    };

    [Tooltip("Optional: assign BirdSpawn_* transforms from the scene")]
    public Transform[] sceneBirdSpawns;

    [Tooltip("Fallback if no scene markers (matches field_layout.yaml)")]
    public BirdSpawn[] birds = new BirdSpawn[]
    {
        new BirdSpawn { rosX = 7f, rosY = 5f, rosZ = 4f, dx = -1f, dy = -0.25f },
        new BirdSpawn { rosX = -6f, rosY = 2f, rosZ = 4f, dx = 1f, dy = -0.2f },
        new BirdSpawn { rosX = 4f, rosY = -7f, rosZ = 4f, dx = -0.5f, dy = 0.6f },
    };

    /// <summary>ROS map → Unity world (X, Z, Y).</summary>
    public static Vector3 RosToUnity(float rosX, float rosY, float rosZ)
    {
        return new Vector3(rosX, rosZ, rosY);
    }

    /// <summary>Unity world → ROS map.</summary>
    public static void UnityWorldToRos(Vector3 world, out float rosX, out float rosY, out float rosZ)
    {
        rosX = world.x;
        rosY = world.z;
        rosZ = world.y;
    }

    public int BirdCount
    {
        get
        {
            if (birds == null || birds.Length == 0)
                return 0;
            return Mathf.Min(birds.Length, maxBirds);
        }
    }

    /// <summary>Read BirdSpawn_* children or sceneBirdSpawns into birds[].</summary>
    public void ApplyFromScene()
    {
        var markers = new List<Transform>();

        if (sceneBirdSpawns != null)
        {
            foreach (var t in sceneBirdSpawns)
            {
                if (t != null)
                    markers.Add(t);
            }
        }

        if (markers.Count == 0)
        {
            foreach (Transform child in transform)
            {
                if (child.name.StartsWith("BirdSpawn"))
                    markers.Add(child);
            }
        }

        if (markers.Count == 0)
            return;

        if (markers.Count > maxBirds)
            markers = markers.GetRange(0, maxBirds);

        var next = new List<BirdSpawn>();
        for (int i = 0; i < markers.Count; i++)
        {
            UnityWorldToRos(markers[i].position, out float rx, out float ry, out float rz);
            float lim = fieldXY > 0f ? fieldXY : limitXY;
            float inset = Mathf.Max(0.5f, lim - 0.5f);
            rx = Mathf.Clamp(rx, -inset, inset);
            ry = Mathf.Clamp(ry, -inset, inset);
            float dx = -1f;
            float dy = 0f;
            if (birds != null && i < birds.Length)
            {
                dx = birds[i].dx;
                dy = birds[i].dy;
            }
            next.Add(new BirdSpawn { rosX = rx, rosY = ry, rosZ = rz, dx = dx, dy = dy });
        }

        birds = next.ToArray();
        Debug.Log($"FieldLayout: loaded {birds.Length} bird spawn(s) from scene transforms.");
    }
}
