# Software Architecture

This diagram is a simplified software architecture view for the Slambot Charlie portfolio entry. It focuses on the main runtime blocks and control/data path rather than listing every ROS topic and service.

```mermaid
%%{init: {
  "theme": "base",
  "themeVariables": {
    "background": "#ffffff",
    "mainBkg": "#ffffff",
    "primaryColor": "#ffffff",
    "secondaryColor": "#ffffff",
    "tertiaryColor": "#ffffff",
    "primaryTextColor": "#000000",
    "secondaryTextColor": "#000000",
    "tertiaryTextColor": "#000000",
    "primaryBorderColor": "#000000",
    "clusterBkg": "#ffffff",
    "clusterBorder": "#000000",
    "lineColor": "#000000",
    "edgeLabelBackground": "#ffffff",
    "fontFamily": "Arial, sans-serif"
  },
  "flowchart": {
    "curve": "linear",
    "htmlLabels": true,
    "nodeSpacing": 45,
    "rankSpacing": 60
  }
}}%%
flowchart LR

    CTRL["Laptop / Phone"] --> NET["Tailnet"]
    NET --> DASH["Web Dashboard"]

    subgraph RPI["Raspberry Pi Software Stack"]
        direction TB
        DASH
        ROS["ROS 2 Runtime"]
        BASE["Base Driver"]
        SLAM["SLAM"]
        LIDAR["LiDAR Driver"]
        CAMERA["Camera Driver"]
        TF["Robot Model / TF"]
    end

    subgraph MCU["Teensy Firmware Stack"]
        direction TB
        SERIAL["USB Serial"]
        MOTOR["Motor Control"]
        ENCODER["Encoder Processing"]
    end

    DASH --> ROS
    ROS --> BASE
    ROS --> SLAM
    LIDAR --> SLAM
    CAMERA --> DASH
    TF --> SLAM

    BASE --> SERIAL
    SERIAL --> MOTOR
    ENCODER --> MOTOR

    classDef block fill:#ffffff,stroke:#000000,color:#000000,stroke-width:1.6px;
    class CTRL,NET,DASH,ROS,BASE,SLAM,LIDAR,CAMERA,TF,SERIAL,MOTOR,ENCODER block;

    style RPI fill:#ffffff,stroke:#000000,color:#000000,stroke-width:1.6px;
    style MCU fill:#ffffff,stroke:#000000,color:#000000,stroke-width:1.6px;
```
