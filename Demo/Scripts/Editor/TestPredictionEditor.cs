using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


namespace Carousel
{

namespace MotionMatching{
[CustomEditor(typeof(TestPrediction))]
public class TestPredictionEditor : Editor

{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        TestPrediction mm = (TestPrediction)target;
        if (GUILayout.Button("search"))
        {

            mm.TestSearch();
        }
    }
}
}
}