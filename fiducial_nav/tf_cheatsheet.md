# TF Cheatsheet

## Introduction

This document gives coding tips for using tfs in ROS. It assumes that the reader
has:

1. studied [the ROS Wiki tf2 tutorial](http://wiki.ros.org/tf2/Tutorials); and
2. understood the basic math behind transforms (e.g., what coordinate frames  
   are, what the translation from one frame to another is, etc.).  


## Prelude

Suppose that we've executed:

```python3
import tf2_ros

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
tf_broadcaster = tf2_ros.TransformBroadcaster()
```

## Tips

**To get** the pose of an existing frame `A`:

```python3
A_tf = tf_buffer.lookup_transform('A', 'A', rospy.Time()).transform
A_translation = A_tf.translation
A_rotation = A_tf.rotation
```

**To write** a node that broadcasts a frame continuously: (i) wrap the  
broadcasting functionality in a `while` loop with the `try-except` structure  
below, and (ii) don't forget to update the timestamp of your frame.

```python3
while not rospy.is_shutdown():
    try:
        new_tfs = TransformStamped()
        # ...
        # Configure new_tfs here.
        # ...
        new_tfs.header.stamp = rospy.Time.now()
    except (
        tf2_ros.LookupException,
        tf2_ros.ExtrapolationException,
        tf2_ros.ConnectivityException
        ):
        continue
    rate.sleep()
```

**To broadcast** a new frame `C` as a child of `A` that: (i) has the same  
rotation as `A`, and (ii) shares its origin with frame `B` (that is part of the  
same tf tree as `A`):

```python3
C_tfs = TransformStamped()
C_tfs.header.frame_id = 'A'
C_tfs.child_frame_id = 'C'

while not rospy.is_shutdown():
    try:
        A_tf = tf_buffer.lookup_transform('A', 'A', rospy.Time()).transform
        A_to_B_tf = tf_buffer.lookup_transform('A', 'B', rospy.Time()).transform
        C_tfs.transform.rotation = A_tf.rotation
        C_tfs.transform.translation = A_to_B_tf.translation
        C_tfs.header.stamp = rospy.Time.now()
    except (
        tf2_ros.LookupException,
        tf2_ros.ExtrapolationException,
        tf2_ros.ConnectivityException
        ):
        continue
    rate.sleep()
```

**To get** the distance between the origin of frame `A` and that of frame `B`:

```python3
import numpy as np
A_to_B_transl = tf_buffer.lookup_transform(
    'A',
    'B',
    rospy.Time()
).transform.translation

dist = np.linalg.norm(
    np.array(
        [
            A_to_B_transl.x,
            A_to_B_transl.y,
            A_to_B_transl.z,
        ]
    )
)
```
