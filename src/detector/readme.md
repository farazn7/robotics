#  Detector Package - Usage Instructions

##  How to Run the Detector Node

In the terminal, run the following command:

```bash
ros2 run detector detector_node
```

---

##  Terminal Output

After launching, the terminal will display messages such as:

1. `Node has started`  
2. `Received image frame`  
3. `Contours Area detected: <value>`  

These logs indicate that the image is being processed and anomalies are being analyzed.

---


## Running the node will open **three popup windows**:

---

### 1.  Tracking

This window allows you to chnage **HSV (Hue, Saturation, Value)** thresholds.

To detect anomalies:
- **Lower Hue**: Set to `5`
- **Lower Value**: Set between `41` and `50`

Adjusting these values helps isolate anomalies based on their color.

---

### 2.  Input Image

Displays the image published by the topic:

```bash
/front_cam/zed_node/rgb/image_rect_color
```

---

### 3. Mask

Displays a binary mask where anomalies appear as **white patches**.

These white areas correspond to the tuned HSV range defined in the **Tracking** window.

---

