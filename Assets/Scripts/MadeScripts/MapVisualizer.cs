using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav; // For OccupancyGridMsg

public class MapVisualizer : MonoBehaviour
{
    public GameObject mapCellPrefab;
    public Transform mapOrigin;
    public float cellSize = 0.05f;

    private ROSConnection ros;
    private Dictionary<Vector2Int, GameObject> cellObjects = new Dictionary<Vector2Int, GameObject>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OccupancyGridMsg>("/map", RenderMap);
    }

    void RenderMap(OccupancyGridMsg map)
    {
        // Set mapOrigin position and rotation from map.info.origin
        mapOrigin.position = new Vector3(
            (float)map.info.origin.position.x,
            0,
            (float)map.info.origin.position.y
        );
        mapOrigin.rotation = new Quaternion(
            (float)map.info.origin.orientation.x,
            (float)map.info.origin.orientation.z,
            (float)map.info.origin.orientation.y,
            (float)map.info.origin.orientation.w
        );

        int width = (int)map.info.width;
        int height = (int)map.info.height;
        float resolution = map.info.resolution;

        // Track which cells are still occupied this update
        HashSet<Vector2Int> occupiedThisFrame = new HashSet<Vector2Int>();

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = y * width + x;
                int value = (int)map.data[index];

                if (value > 0)
                {
                    Vector2Int cell = new Vector2Int(x, y);
                    occupiedThisFrame.Add(cell);

                    if (!cellObjects.ContainsKey(cell))
                    {
                        Vector3 position = new Vector3(
                            x * resolution,
                            0,
                            y * resolution
                        );
                        GameObject obj = Instantiate(mapCellPrefab, mapOrigin.TransformPoint(position), Quaternion.identity, mapOrigin);
                        cellObjects[cell] = obj;
                    }
                }
            }
        }

        // Remove cells that are no longer occupied
        var cellsToRemove = new List<Vector2Int>();
        foreach (var kvp in cellObjects)
        {
            if (!occupiedThisFrame.Contains(kvp.Key))
            {
                Destroy(kvp.Value);
                cellsToRemove.Add(kvp.Key);
            }
        }
        foreach (var cell in cellsToRemove)
        {
            cellObjects.Remove(cell);
        }
    }
}