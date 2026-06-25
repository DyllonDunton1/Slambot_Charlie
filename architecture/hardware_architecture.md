# Hardware Architecture

This diagram is a simplified hardware architecture view for the Slambot Charlie portfolio entry. It intentionally shows the main system relationships without listing every connector, pin, or part-specific detail.

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
    "nodeSpacing": 55,
    "rankSpacing": 70
  }
}}%%
flowchart LR

    subgraph POWER["Power System"]
        direction TB
        BAT["Battery"]
        V5["5V Line"]
        V12["12V Line"]
        BAT --> V5
        BAT --> V12
    end

    subgraph RPI["Raspberry Pi Hardware"]
        direction TB
        PI["Raspberry Pi 4B"]
        LIDAR["LiDAR"]
        CAMERA["USB Camera"]
        IMU["IMU"]
        LIDAR --> PI
        CAMERA --> PI
        IMU --> PI
    end

    USB["USB Power + Serial Data"]

    subgraph MCU["Teensy Hardware"]
        direction TB
        TEENSY["Teensy 4.1"]
        ENCODER["Wheel Encoders"]
        ENCODER --> TEENSY
    end

    subgraph DRIVE["Drive Hardware"]
        direction TB
        DRIVER["Stepper Drivers"]
        MOTOR["Stepper Motors"]
        DRIVER --> MOTOR
    end

    V5 --> PI
    V12 --> DRIVER
    PI --> USB
    USB --> TEENSY
    TEENSY --> DRIVER

    classDef block fill:#ffffff,stroke:#000000,color:#000000,stroke-width:1.6px;
    class BAT,V5,V12,PI,LIDAR,CAMERA,IMU,USB,TEENSY,ENCODER,DRIVER,MOTOR block;

    style POWER fill:#ffffff,stroke:#000000,color:#000000,stroke-width:1.6px;
    style RPI fill:#ffffff,stroke:#000000,color:#000000,stroke-width:1.6px;
    style MCU fill:#ffffff,stroke:#000000,color:#000000,stroke-width:1.6px;
    style DRIVE fill:#ffffff,stroke:#000000,color:#000000,stroke-width:1.6px;
```
