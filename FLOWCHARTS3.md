# Flowchart 4: Kill Switch & Safety Handling

## Kill Switch Priority

```mermaid
flowchart TD
    A[every loop iteration] --> B[loopWifi\nprocess MQTT messages]
    A --> C[checkKillButton\nhardware pin 39]

    B --> D{heartbeat timeout?\n> 1000ms since last}
    D -- yes --> KILL[systemEnabled = false\nKILLED]
    B --> E{emergency message?}
    E -- yes --> KILL
    B --> F{disable message?\nenabled=false}
    F -- yes --> KILL

    C --> G{button pressed?\ndebounced}
    G -- yes --> H[toggle isKilledLocal]

    KILL --> I[killed check in loop]
    H --> I
    I --> J{isKilledLocal OR\n!isSystemEnabled?}
    J -- yes --> K[stopTracks\nstopPlanter]
    K --> L[blink red LED]
    L --> M[skip all autonomous logic]

    J -- no --> N[solid green LED]
    N --> O[run autonomous logic]
```

## Kill Switch Sources

```mermaid
flowchart LR
    HW[Hardware Button\npin 39\ntoggle on press] --> KILLED{killed?}
    MQTT_D[WiFi disable\nmessage] --> KILLED
    MQTT_E[WiFi emergency\nmessage] --> KILLED
    HB[Heartbeat timeout\nserver lost > 1s] --> KILLED

    KILLED -- yes --> STOP[stopTracks\nstopPlanter\nblink red LED]
    KILLED -- no --> RUN[run autonomous\nsolid green LED]
```

## Button Debounce Logic

```mermaid
flowchart TD
    A[read pin 39] --> B{reading changed\nfrom last?}
    B -- yes --> C[reset debounce timer]
    B -- no --> D{timer > 50ms?}
    D -- no --> END[ignore, keep current state]
    D -- yes --> E{reading differs\nfrom killBtnState?}
    E -- no --> END
    E -- yes --> F[update killBtnState]
    F --> G{button LOW?\nactive-low, pull-up}
    G -- yes --> H[toggle isKilledLocal]
    H --> I{now killed?}
    I -- yes --> J[stopTracks immediately]
    I -- no --> K[resume operation]
    G -- no --> END
```

# Flowchart 5: WiFi/MQTT Communication

## Message Handling

```mermaid
flowchart TD
    A[MQTT message received] --> B{valid message?\nprintable characters}
    B -- no --> DROP[discard]
    B -- yes --> C{length == 6?}
    C -- yes --> D[binary status flags\nairlock/emergency/reentry]
    C -- no --> E{length == 21?}
    E -- yes --> F[occupancy map]
    E -- no --> G[parse key=value pairs]

    G --> H{type field?}
    H -- disable --> I[set systemEnabled\nfrom enabled field]
    H -- emergency --> J[systemEnabled = false]
    H -- isFertileReply --> K[invoke fertilityCallback\nfertile true/false]
    H -- heartbeat --> L[reset heartbeat timer\nupdate systemEnabled]
    H -- openAirlockReply --> M[airlock acknowledged]
    H -- no type --> DROP
```

## Registration & Heartbeat

```mermaid
flowchart TD
    A[loopWifi called every loop] --> B[messenger.loop\nprocess MQTT]
    B --> C{> 5s since\nlast register?}
    C -- yes --> D[re-register with server\ntype=register team_id board_id]
    C -- no --> E{systemEnabled AND\nheartbeat timeout?}
    E -- yes --> F[systemEnabled = false\nserver connection lost]
    E -- no --> G[continue]
```
