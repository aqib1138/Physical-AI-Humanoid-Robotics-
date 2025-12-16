# Data Model: Vision-Language-Action (VLA) for Physical AI & Humanoid Robotics

## Conceptual Data Model

### Voice-to-Action Pipeline Entity

**Description**: The system that processes speech input through Whisper-based recognition, extracts intent, and maps to ROS 2 commands for robotic execution

**Attributes**:
- voice_input: AudioStream - Raw audio data from microphone
- transcribed_text: String - Text output from Whisper ASR
- extracted_intent: SemanticIntent - Parsed meaning from natural language
- ros_command: ROSCommand - Mapped ROS 2 action/service/topic message
- confidence_score: Float - Confidence level of intent extraction
- processing_timestamp: DateTime - When the processing occurred

**Relationships**:
- One Voice-to-Action Pipeline receives many VoiceInputs
- One Voice-to-Action Pipeline generates one ROSCommand per input
- Connected to ROS 2 system for command execution

### Cognitive Planning System Entity

**Description**: The LLM-based system that translates high-level natural language goals into detailed ROS 2 action sequences with task decomposition

**Attributes**:
- high_level_goal: String - Natural language goal description
- task_decomposition: TaskSequence - Breakdown of goal into subtasks
- action_sequence: ROSAction[] - Sequence of ROS 2 actions to execute
- context_state: RobotState - Current state of robot and environment
- safety_constraints: SafetyRule[] - Safety rules to be enforced
- planning_timestamp: DateTime - When the plan was generated

**Relationships**:
- One Cognitive Planning System processes many HighLevelGoals
- One Cognitive Planning System generates many ActionSequences
- Connected to RobotState for context awareness

### Autonomous Humanoid System Entity

**Description**: The integrated system combining navigation, perception, manipulation, and cognitive planning for complete autonomous behavior

**Attributes**:
- current_behavior: BehaviorState - Current operational state
- sensor_data: SensorReading[] - Data from all robot sensors
- navigation_goals: NavigationGoal[] - Waypoints and destinations
- manipulation_targets: ObjectReference[] - Objects to interact with
- interaction_history: InteractionLog[] - History of human-robot interactions
- system_status: SystemStatus - Overall system health and status

**Relationships**:
- One Autonomous Humanoid System integrates all VLA components
- Many SensorReadings feed into the system
- Many BehaviorStates represent different operational modes

## Data Flow Patterns

### Voice Processing Flow
Voice Input → Whisper ASR → Intent Extraction → ROS Command Mapping → Robot Execution

### Planning Flow
Natural Language Goal → LLM Processing → Task Decomposition → Action Sequencing → Execution

### Integration Flow
Sensor Data → Perception → Cognitive Planning → Action Selection → Robot Control

## State Models

### Voice Processing States
- IDLE: Awaiting voice input
- LISTENING: Capturing audio
- PROCESSING: Running ASR and intent extraction
- COMMANDING: Executing ROS command
- ERROR: Processing failure state

### Planning States
- RECEIVING_GOAL: Accepting natural language input
- ANALYZING: Understanding goal requirements
- DECOMPOSING: Breaking down into subtasks
- VALIDATING: Checking safety constraints
- EXECUTING_PLAN: Running action sequence

### System States
- IDLE: Waiting for commands
- ACTIVE: Processing tasks
- NAVIGATING: Moving to destination
- MANIPULATING: Handling objects
- RECOVERING: Error recovery mode
- SAFETY_LOCKOUT: Safety constraint activated