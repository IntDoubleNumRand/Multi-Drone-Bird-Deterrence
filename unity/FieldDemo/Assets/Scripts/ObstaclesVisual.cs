// Static obstacle cylinders from field layout (matches ROS /obstacles/positions).
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class ObstaclesVisual : MonoBehaviour
{
    public FieldLayout layout;
    public Material obstacleMaterial;
    public float height = 2f;

    void Awake()
    {
        if (layout == null)
            layout = GetComponentInParent<FieldLayout>();
        if (layout == null)
            layout = FindObjectOfType<FieldLayout>();

        BuildObstacles();
    }

    void BuildObstacles()
    {
        if (layout == null || layout.obstacles == null)
            return;

        int n = Mathf.Min(layout.obstacles.Length, layout.maxObstacles > 0 ? layout.maxObstacles : layout.obstacles.Length);
        for (int i = 0; i < n; i++)
        {
            var o = layout.obstacles[i];
            var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = string.IsNullOrEmpty(o.name) ? $"Obstacle_{i}" : o.name;
            go.transform.SetParent(transform, false);
            float diameter = o.radius * 2f;
            go.transform.localScale = new Vector3(diameter, height * 0.5f, diameter);
            go.transform.position = FieldLayout.RosToUnity(o.rosX, o.rosY, height * 0.5f);

            var col = go.GetComponent<Collider>();
            if (col != null)
                Destroy(col);

            var r = go.GetComponent<Renderer>();
            if (r != null)
            {
                if (obstacleMaterial != null)
                    r.sharedMaterial = obstacleMaterial;
                else
                    r.material.color = new Color(0.45f, 0.45f, 0.45f, 0.85f);
            }
        }
    }
}
