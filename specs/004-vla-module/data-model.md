# Data Model: Vision-Language-Action (VLA) Module

## Overview
This document defines the conceptual data model for the Vision-Language-Action (VLA) system described in the educational module. These concepts help students understand the key data structures and relationships in VLA systems.

## Core Entities

### Voice Command
- **Description**: Natural language input from users that specifies desired robot behavior
- **Attributes**:
  - `command_text` (string): The transcribed text of the voice command
  - `timestamp` (datetime): When the command was issued
  - `confidence` (float): Confidence level of speech recognition (0.0-1.0)
  - `user_id` (string): Identifier for the user issuing the command
  - `context` (object): Environmental context when command was issued

### Intent Mapping
- **Description**: The process of converting natural language to specific robot actions
- **Attributes**:
  - `source_command` (string): Original command text
  - `detected_intent` (string): Identified intent from the command
  - `entities` (array): Identified entities in the command (objects, locations, etc.)
  - `confidence` (float): Confidence in intent classification (0.0-1.0)
  - `mapped_action` (string): The robot action to execute

### Action Plan
- **Description**: A sequence of executable steps that accomplish a user's high-level goal
- **Attributes**:
  - `plan_id` (string): Unique identifier for the plan
  - `tasks` (array): Sequence of tasks to execute
  - `status` (enum): Current status (pending, executing, completed, failed)
  - `created_at` (datetime): When the plan was created
  - `estimated_duration` (float): Estimated time to complete the plan
  - `dependencies` (array): Dependencies between tasks

### Task
- **Description**: An individual executable unit within an action plan
- **Attributes**:
  - `task_id` (string): Unique identifier for the task
  - `action_type` (enum): Type of action (navigation, perception, manipulation)
  - `parameters` (object): Parameters needed for the action
  - `status` (enum): Current status (pending, executing, completed, failed)
  - `estimated_time` (float): Estimated time to complete the task
  - `prerequisites` (array): Prerequisites that must be met before execution

### Safety Boundary
- **Description**: Constraints that prevent the robot from executing potentially harmful actions
- **Attributes**:
  - `boundary_type` (enum): Type of boundary (physical, operational, ethical)
  - `parameters` (object): Parameters defining the boundary
  - `severity` (enum): Severity level (warning, error, critical)
  - `override_conditions` (array): Conditions under which boundary can be overridden
  - `last_check` (datetime): When the boundary was last evaluated

### Task Context
- **Description**: Information about the current task state, environment, and object locations
- **Attributes**:
  - `current_task` (string): Identifier of the currently executing task
  - `environment_state` (object): Current state of the environment
  - `object_locations` (object): Locations of known objects
  - `robot_state` (object): Current state of the robot
  - `timestamp` (datetime): When the context was last updated

## Relationships

### Voice Command → Intent Mapping
- **Relationship**: One-to-One
- **Description**: Each voice command maps to one intent mapping
- **Rule**: A command must have exactly one intent mapping before execution

### Intent Mapping → Action Plan
- **Relationship**: One-to-One
- **Description**: Each intent mapping results in one action plan
- **Rule**: An intent mapping must generate exactly one action plan

### Action Plan → Task
- **Relationship**: One-to-Many
- **Description**: Each action plan consists of multiple tasks
- **Rule**: An action plan must have at least one task

### Task → Safety Boundary
- **Relationship**: Many-to-Many
- **Description**: Tasks may be subject to multiple safety boundaries
- **Rule**: Each task must pass all applicable safety boundary checks

### Task → Task Context
- **Relationship**: One-to-One
- **Description**: Each task has a context that tracks its execution state
- **Rule**: A task's context is updated throughout its execution

## State Transitions

### Task States
- `pending` → `executing`: When task execution begins
- `executing` → `completed`: When task completes successfully
- `executing` → `failed`: When task execution fails
- `executing` → `interrupted`: When task is interrupted by safety boundary

### Action Plan States
- `pending` → `executing`: When first task begins execution
- `executing` → `completed`: When all tasks complete successfully
- `executing` → `failed`: When any task fails and no recovery is possible
- `executing` → `interrupted`: When safety boundary stops execution

## Validation Rules

1. **Command Validity**: Voice commands must be non-empty and have confidence > 0.5
2. **Intent Confidence**: Intent mappings must have confidence > 0.7 to be accepted
3. **Task Sequence**: Tasks in an action plan must respect dependency order
4. **Safety Check**: All tasks must pass safety boundary checks before execution
5. **Context Freshness**: Task context must be updated within 5 seconds of execution
6. **Plan Feasibility**: Action plans must be physically feasible for the robot
7. **Boundary Override**: Safety boundaries can only be overridden by authorized users