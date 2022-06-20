using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


namespace Carousel
{

namespace MotionMatching{
[CustomEditor(typeof(LocomotionController))]
public class LocomotionControllerEditor : Editor

{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        LocomotionController c = (LocomotionController)target;
    
        if (GUILayout.Button("find transition"))
        {

           c.FindTransition();
        }
    }
}
}
}