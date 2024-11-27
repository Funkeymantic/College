%%{
  init: {
    'theme': 'base',
    'themeVariables': {
      'primaryColor': '#000000',
      'primaryTextColor': '#ffffff',
      'primaryBorderColor': '#707070',
      'lineColor': '#292929',
      'secondaryColor': '#707070',
      'tertiaryColor': '#707070'
    }
  }
}%%
flowchart TD
%% Nodes
    A("Establish communication with MQTT Broker")
    B("Establish connection with Camera")
    C("Capture & Save image")
    D("Find Robot Coordinates from image")
    E("Setup dice capture playlist")
    F("Publish 'Ready'")
    G("Home Robot (Joint)")
    H("Wait for 'Ready' message")
    I("Initiate playlist")
    J("Publish 'Standby'")
    K("Capture Image")
    L("Find Shared Robot Coordinates")
    M("Setup playlist (Even pip dice)")
    N("Move home")
    O("Publish 'Complete")
    P("Move to Standby Position")
    Q("Wait for 'Complete'")
    R("Capture Image")
    S("Find Shared Coordinates")
    T("Set up playlist")
    U("Initiate playlist")
    V("Move Home")
    W("Publish 'Complete'")
    X("Wait for 'Complete'")
    Y("Disconnect from MQTT Broker")
    Z("End Program")

%% Edge connections between nodes
    A --> B --> C --> D --> E --> F --> G
    G --> |IF READY = FALSE| H -->
    G --> |IF READY = TRUE| I --> 
    |IF SPECIFIED DICE REMOVED|J
    J --> |IF STANDBY = FALSE|K --> L --> M --> N --> O
    J --> |IF STNDABY = TRUE|P --> Q --> R --> S --> T --> U --> V --> W
    O --> X
    W --> X
    X --> Y --> Z

%% Individual node styling. Try the visual editor toolbar for easier styling!
    

%% You can add notes with two "%" signs in a row!