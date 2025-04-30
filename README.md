# VideoCall Vacuum
Turn your old Roomba into a telepresence robot! A Raspberry Pi streams camera and sensor data to a Laptop, which uses computer vision to detect and follow people. The Laptop sends movement commands back to the Pi, making your Roomba smart and social!

<div align="center">
    <img src="image.png" alt="alt text" width="25%">
</div>

## Top Level
eCAL used for communication between Laptop and Raspberry Pi
```mermaid
graph TD
    subgraph Laptop
        A[VideoCallVacuum Main]
        B[PersonDetector]
        C[PersonFollower]
    end

    subgraph Raspberry Pi
        D[WebcamPublisher]
        E[SensorsPublisher]
        F[MovementSubscriber]
        D -->|webcam_feed| A
        E -->|sensors| A
        A -->|movement_command| F
    end
```

## Person Detector
```mermaid
graph TD
    A[Start] --> B[Initialize Kalman Filter and YOLO Model]
    B --> C[Receive Image Frame]
    C --> D[Convert Frame to Grayscale]
    D --> E{Previous Frame Exists?}
    E -->|Yes| F[Calculate Optical Flow]
    E -->|No| G[Skip Optical Flow]
    F --> H[Draw Optical Flow on Image]
    G --> H
    H --> I{Run YOLO Detection?}
    I -->|Yes| J[Create Blob and Run YOLO]
    I -->|No| K[Use Previous Detections]
    J --> L[Process YOLO Outputs]
    K --> L
    L --> M[Filter Detections by Confidence and Class ID]
    M --> N{Person Found?}
    N -->|Yes| O[Find Most Centered Person]
    O --> P[Update Kalman Filter with Detected Position]
    P --> Q[Return Person Box and Position]
    N -->|No| R[Predict Position Using Kalman Filter]
    R --> S[Return Predicted Position]
    Q --> T[End]
    S --> T
```

## Person Follower
```mermaid
stateDiagram-v2
    [*] --> FIND_SOMEONE
    state FIND_SOMEONE {
        WallFollowing: Follow wall to the right
    }
    FIND_SOMEONE --> PERSON_IN_FRAME: Person Found

    state PERSON_IN_FRAME {
        TrackPerson: Track person and follow
    }
    PERSON_IN_FRAME --> PERSON_BEHIND: Person Behind
    PERSON_IN_FRAME --> PERSON_SIDEWAYS: Person Sideways
    PERSON_IN_FRAME --> PERSON_AWAY: Person Away
    PERSON_IN_FRAME --> FIND_SOMEONE: Person Lost for Threshold Time

    state PERSON_BEHIND {
        Turn360: Turn 360 degrees towards last known position
    }
    PERSON_BEHIND --> PERSON_IN_FRAME: Person Found
    PERSON_BEHIND --> FIND_SOMEONE: Threshold Time Exceeded

    state PERSON_SIDEWAYS {
        Turn180: Turn up to 180 degrees towards last known position
    }
    PERSON_SIDEWAYS --> PERSON_IN_FRAME: Person Found
    PERSON_SIDEWAYS --> FIND_SOMEONE: Threshold Time Exceeded

    state PERSON_AWAY {
        MoveForwardOrTurn: Move forward or turn towards last known position
    }
    PERSON_AWAY --> PERSON_IN_FRAME: Person Found
    PERSON_AWAY --> FIND_SOMEONE: Threshold Time Exceeded

    PERSON_AWAY --> FIND_SOMEONE: Turn Threshold Exceeded
```

