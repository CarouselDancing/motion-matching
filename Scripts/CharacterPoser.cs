using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

abstract public class CharacterPoser : MonoBehaviour
{
    public bool initialized = false;
    public bool active = true;
    public bool inPipeline = false;
    abstract public void SetTransforms();

    abstract public void UpdatePose();
}