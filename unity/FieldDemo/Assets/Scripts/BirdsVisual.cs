// Moves scene birds from /birds/positions. Colors from /birds/status + chase topics.
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using geometry_msgs.msg;
using std_msgs.msg;
using Vector3 = UnityEngine.Vector3;
using Transform = UnityEngine.Transform;
using Quaternion = UnityEngine.Quaternion;

public class BirdsVisual : ROS2UnityComponent
{
    public FieldLayout layout;  // auto-filled from BirdsRoot FieldLayout if empty
    public GameObject birdPrefab;

    [Header("Bird status colors (match /birds/status)")]
    public Color wanderColor = new Color(1f, 0.45f, 0.05f);
    public Color fleeColor = new Color(1f, 0.1f, 0.1f);
    public Color recoverColor = new Color(0.2f, 0.75f, 1f);
    public Color enterColor = new Color(0.35f, 0.9f, 0.4f);
    public Color targetColor = new Color(1f, 0.85f, 0.1f);

    private readonly List<Transform> slots = new List<Transform>();
    private readonly List<Renderer> renderers = new List<Renderer>();
    private readonly List<MaterialPropertyBlock> blocks = new List<MaterialPropertyBlock>();
    private ROS2Node ros2Node;

    private readonly object birdsLock = new object();
    private Vector3[] latestBirds;
    private bool hasBirds;

    private int[] birdStates;
    private bool isChased;
    private int targetIndex = -1;
    private string lastStatusKey = "";

    void Awake()
    {
        if (layout == null)
            layout = GetComponent<FieldLayout>();
        if (layout == null)
            layout = gameObject.AddComponent<FieldLayout>();

        layout.ApplyFromScene();
        SetupSlots();
    }

    void SetupSlots()
    {
        slots.Clear();
        renderers.Clear();
        blocks.Clear();

        int n = layout != null ? layout.BirdCount : 0;
        if (n < 1)
        {
            Debug.LogWarning("BirdsVisual: no FieldLayout birds defined.");
            return;
        }

        var spawnMarkers = new List<Transform>();
        foreach (Transform child in transform)
        {
            if (!child.name.StartsWith("BirdSpawn"))
                continue;
            spawnMarkers.Add(child);
        }
        spawnMarkers.Sort((a, b) => string.CompareOrdinal(a.name, b.name));

        for (int i = 0; i < spawnMarkers.Count && slots.Count < n; i++)
        {
            Transform child = spawnMarkers[i];
            Transform slot = child;
            if (birdPrefab != null && child.childCount == 0)
            {
                var go = Instantiate(birdPrefab, child);
                go.transform.localPosition = Vector3.zero;
                go.transform.localRotation = Quaternion.identity;
                slot = go.transform;
            }
            slots.Add(slot);
        }

        for (int i = n; i < spawnMarkers.Count; i++)
            spawnMarkers[i].gameObject.SetActive(false);

        if (slots.Count == 0)
        {
            for (int i = 0; i < n; i++)
            {
                var go = birdPrefab != null
                    ? Instantiate(birdPrefab, transform)
                    : GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go.name = $"BirdSlot{i}";
                slots.Add(go.transform);
            }
        }
        else if (slots.Count > n)
        {
            while (slots.Count > n)
            {
                var t = slots[slots.Count - 1];
                slots.RemoveAt(slots.Count - 1);
                if (Application.isPlaying)
                    Destroy(t.gameObject);
            }
        }

        for (int i = 0; i < n; i++)
        {
            var spawn = layout.birds[i];
            slots[i].position = FieldLayout.RosToUnity(spawn.rosX, spawn.rosY, spawn.rosZ);
            var r = slots[i].GetComponentInChildren<Renderer>();
            renderers.Add(r);
            blocks.Add(new MaterialPropertyBlock());
        }

        ApplyColors();
    }

    void Start()
    {
        ros2Node = CreateNode("unity_birds_viz");
        var qos = new QualityOfServiceProfile(QosPresetProfile.DEFAULT);

        ros2Node.CreateSubscription<PoseArray>("/birds/positions", OnBirds, qos);
        ros2Node.CreateSubscription<Int32MultiArray>("/birds/status", OnStatus, qos);
        ros2Node.CreateSubscription<Bool>("/bird/chased", OnChased, qos);
        ros2Node.CreateSubscription<Int32>("/coordinator/target_index", OnTarget, qos);
    }

    void OnBirds(PoseArray msg)
    {
        int maxBirds = layout != null ? layout.BirdCount : slots.Count;
        int n = Mathf.Min(slots.Count, msg.Poses.Length, maxBirds);
        var arr = new Vector3[n];
        for (int i = 0; i < n; i++)
        {
            var p = msg.Poses[i].Position;
            arr[i] = new Vector3((float)p.X, (float)p.Z, (float)p.Y);
        }
        lock (birdsLock)
        {
            latestBirds = arr;
            hasBirds = true;
        }
    }

    void OnStatus(Int32MultiArray msg)
    {
        if (msg.Data == null || msg.Data.Length == 0)
        {
            birdStates = null;
            return;
        }
        birdStates = new int[msg.Data.Length];
        for (int i = 0; i < msg.Data.Length; i++)
            birdStates[i] = msg.Data[i];
    }

    void OnChased(Bool msg) => isChased = msg.Data;

    void OnTarget(Int32 msg) => targetIndex = msg.Data;

    void Update()
    {
        if (!hasBirds || latestBirds == null) return;

        Vector3[] arr;
        lock (birdsLock)
            arr = latestBirds;

        for (int i = 0; i < arr.Length && i < slots.Count; i++)
            slots[i].position = arr[i];

        string key = StatusKey();
        if (key != lastStatusKey)
        {
            lastStatusKey = key;
            ApplyColors();
        }
    }

    string StatusKey()
    {
        var parts = new List<string> { isChased ? "1" : "0", targetIndex.ToString() };
        if (birdStates != null)
        {
            foreach (int s in birdStates)
                parts.Add(s.ToString());
        }
        return string.Join(",", parts);
    }

    Color ColorForBird(int i)
    {
        if (isChased && i == targetIndex)
            return targetColor;

        if (birdStates != null && i < birdStates.Length)
        {
            switch (birdStates[i])
            {
                case 1: return fleeColor;
                case 2: return recoverColor;
                case 3: return enterColor;
                default: return wanderColor;
            }
        }
        return wanderColor;
    }

    void ApplyColors()
    {
        for (int i = 0; i < renderers.Count; i++)
        {
            if (renderers[i] == null) continue;
            var block = blocks[i];
            block.SetColor("_Color", ColorForBird(i));
            block.SetColor("_BaseColor", ColorForBird(i));
            renderers[i].SetPropertyBlock(block);
        }
    }
}
