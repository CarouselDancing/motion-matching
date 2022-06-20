using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace Carousel
{

namespace MotionMatching{
[CustomEditor(typeof(MotionMatching))]
public class MotionMatchingEditor : Editor

{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        MotionMatching mm = (MotionMatching)target;
        if (GUILayout.Button("load"))
        {

            mm.Load();
        }
        if (GUILayout.Button("compute features"))
        {

            mm.ComputeFeatures();
        }
    }
}
}
}