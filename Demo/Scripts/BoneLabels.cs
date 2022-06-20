using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


namespace Carousel
{

namespace MotionMatching{
    
public class BoneLabels : MonoBehaviour
{
    public MMPoseProvider poseProvider;

    public List<RectTransform> labels;
    public RectTransform canvasRect;
    public GameObject labelPrefab;
    public Vector3 offset;

    // Update is called once per frame
    void Update()
    {
        if (labels.Count == 0 && poseProvider.mm.initialized)
        {
            CreateMarkers();
        }
        Vector3 position;
        for (int i = 0; i < labels.Count; i++)
        {
           var bone = poseProvider.mm.database.boneArray[i];
           poseProvider.GetGlobalPosition(bone, out position);
           updateMarker(i, position);
        }
    }

    void CreateMarkers()
    {
        labels = new List<RectTransform>();
        foreach (var label in poseProvider.mm.database.boneNames)
        {
            var o = GameObject.Instantiate(labelPrefab, canvasRect);
            o.name = label;
            var text = o.GetComponent<Text>();
            text.text = label;
            labels.Add(o.GetComponent<RectTransform>());
        }
    }


    //https://forum.unity.com/threads/create-ui-health-markers-like-in-world-of-tanks.432935/#post-2800360
    void updateMarker(int i, Vector3 position)
    {
    

        // Final position of marker above GO in world space
        Vector3 offsetPos = position + offset;

        // Calculate *screen* position (note, not a canvas/recttransform position)
        Vector2 screenPoint = Camera.main.WorldToScreenPoint(offsetPos);
        Vector2 canvasPos;

        // Convert screen position to Canvas / RectTransform space <- leave camera null if Screen Space Overlay
        RectTransformUtility.ScreenPointToLocalPointInRectangle(canvasRect, screenPoint, null, out canvasPos);

        // Set
        labels[i].localPosition = canvasPos;
    }

}

}
}